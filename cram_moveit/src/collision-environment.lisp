;;; Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Universitaet Bremen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-moveit)

(defclass collision-object ()
  ((name :initform nil
         :initarg :name
         :reader :name)
   (primitive-shapes :initform nil
                     :initarg :primitive-shapes
                     :reader :primitive-shapes)
   (mesh-shapes :initform nil
                :initarg :mesh-shapes
                :reader :mesh-shapes)
   (plane-shapes :initform nil
                 :initarg :plane-shapes
                 :reader :plane-shapes)))

(defvar *known-collision-objects* nil
  "List of collision object instances registered with the CRAM/MoveIt!
bridge.")

(defun register-collision-object (name
                                  &key
                                    primitive-shapes
                                    mesh-shapes
                                    plane-shapes)
  (unless (named-collision-object name)
    (push
     (make-instance 'collision-object
                    :name name
                    :primitive-shapes primitive-shapes
                    :mesh-shapes mesh-shapes
                    :plane-shapes plane-shapes)
     *known-collision-objects*)))

(defun unregister-collision-object (name)
  (setf *known-collision-objects*
        (remove name *known-collision-objects*
                :test (lambda (name object)
                        (equal name (slot-value object 'name))))))

(defun named-collision-object (name)
  (let ((position (position name *known-collision-objects*
                            :test (lambda (name object)
                                    (equal name (slot-value object 'name))))))
    (when position
      (nth position *known-collision-objects*))))

(defun create-collision-object-message (name pose-stamped 
                                        &key
                                          primitive-shapes
                                          mesh-shapes
                                          plane-shapes)
  (unless (or primitive-shapes mesh-shapes plane-shapes)
    (cpl:fail 'no-collision-shapes-defined))
  (flet* ((resolve-pose (pose-msg)
            (or pose-msg (tf:pose->msg pose-stamped)))
          (pose-present (object)
            (and (listp object) (cdr object)))
          (resolve-object (obj)
            (or (and (listp obj) (car obj)) obj))
          (prepare-shapes (shapes)
            (map 'vector 'identity (mapcar #'resolve-object shapes)))
          (prepare-poses (poses)
            (map 'vector #'resolve-pose (mapcar #'pose-present poses))))
    (let* ((obj-msg (roslisp:make-msg
                     "moveit_msgs/CollisionObject"
                     (stamp header) (tf:stamp pose-stamped)
                     (frame_id header) (tf:frame-id pose-stamped)
                     id name
                     operation (roslisp-msg-protocol:symbol-code
                                'moveit_msgs-msg:collisionobject
                                :add)
                     primitives (prepare-shapes primitive-shapes)
                     primitive_poses (prepare-poses primitive-shapes)
                     meshes (prepare-shapes mesh-shapes)
                     mesh_poses (prepare-poses mesh-shapes)
                     planes (prepare-shapes plane-shapes)
                     plane_poses (prepare-poses plane-shapes))))
      obj-msg)))

(defun add-collision-object (name pose-stamped)
  (let ((col-obj (named-collision-object name)))
    (when col-obj
      (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
            (mesh-shapes (slot-value col-obj 'mesh-shapes))
            (plane-shapes (slot-value col-obj 'plane-shapes)))
        (let* ((obj-msg (roslisp:modify-message-copy
                         (create-collision-object-message
                          name pose-stamped
                          :primitive-shapes primitive-shapes
                          :mesh-shapes mesh-shapes
                          :plane-shapes plane-shapes)
                         operation (roslisp-msg-protocol:symbol-code
                                    'moveit_msgs-msg:collisionobject
                                    :add)))
               (world-msg (roslisp:make-msg
                           "moveit_msgs/PlanningSceneWorld"
                           collision_objects (vector obj-msg)))
               (scene-msg (roslisp:make-msg
                           "moveit_msgs/PlanningScene"
                           world world-msg
                           is_diff t)))
          (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
            (roslisp:ros-info
             (moveit)
             "Added collision object `~a' to environment server." name)))))))

(defun remove-collision-object (name)
  (let ((col-obj (named-collision-object name)))
    (when col-obj
      (let* ((obj-msg (roslisp:make-msg
                       "moveit_msgs/CollisionObject"
                       id name
                       operation (roslisp-msg-protocol:symbol-code
                                  'moveit_msgs-msg:collisionobject
                                  :remove)))
             (world-msg (roslisp:make-msg
                         "moveit_msgs/PlanningSceneWorld"
                         collision_objects (vector obj-msg)))
             (scene-msg (roslisp:make-msg
                         "moveit_msgs/PlanningScene"
                         world world-msg
                         is_diff t)))
        (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
          (roslisp:ros-info
           (moveit)
           "Removed collision object `~a' from environment server." name))))))

(defun clear-collision-objects ()
  (loop for col-obj in *known-collision-objects*
        do (remove-collision-object (slot-value col-obj 'name))))

(defun attach-collision-object-to-link (name target-link current-pose-stamped
                                        &key touch-links)
  (let ((col-obj (named-collision-object name)))
    (when col-obj
      (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
            (mesh-shapes (slot-value col-obj 'mesh-shapes))
            (plane-shapes (slot-value col-obj 'plane-shapes)))
        (unless (tf:wait-for-transform
                 *tf*
                 :timeout 5.0
                 :time (tf:stamp current-pose-stamped)
                 :source-frame (tf:frame-id current-pose-stamped)
                 :target-frame target-link)
          (cpl:fail 'pose-not-transformable-into-link))
        (let* ((pose-in-link (tf:transform-pose
                              *tf*
                              :pose current-pose-stamped
                              :target-frame target-link))
               (obj-msg-plain (create-collision-object-message
                               name pose-in-link
                               :primitive-shapes primitive-shapes
                               :mesh-shapes mesh-shapes
                               :plane-shapes plane-shapes))
               (obj-msg (roslisp:modify-message-copy
                         obj-msg-plain
                         operation (roslisp-msg-protocol:symbol-code
                                    'moveit_msgs-msg:collisionobject
                                    :remove)))
               (attach-msg (roslisp:make-msg
                            "moveit_msgs/AttachedCollisionObject"
                            link_name target-link
                            object (roslisp:modify-message-copy
                                    obj-msg-plain
                                    operation (roslisp-msg-protocol:symbol-code
                                               'moveit_msgs-msg:collisionobject
                                               :add)
                                    id name)
                            touch_links (concatenate
                                         'vector
                                         (list target-link) touch-links)
                            weight 1.0))
               (world-msg (roslisp:make-msg
                           "moveit_msgs/PlanningSceneWorld"
                           collision_objects (vector obj-msg)))
               (scene-msg (roslisp:make-msg
                           "moveit_msgs/PlanningScene"
                           world world-msg
                           (attached_collision_objects robot_state) (vector
                                                                     attach-msg)
                           is_diff t)))
          (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
            (roslisp:ros-info
             (moveit)
             "Attached collision object `~a' to link `~a'."
             name target-link)))))))

(defun detach-collision-object-from-link (name target-link current-pose-stamped)
  (let ((col-obj (named-collision-object name)))
    (when col-obj
      (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
            (mesh-shapes (slot-value col-obj 'mesh-shapes))
            (plane-shapes (slot-value col-obj 'plane-shapes)))
        (unless (tf:wait-for-transform
                 *tf*
                 :timeout 5.0
                 :time (tf:stamp current-pose-stamped)
                 :source-frame (tf:frame-id current-pose-stamped)
                 :target-frame target-link)
          (cpl:fail 'pose-not-transformable-into-link))
        (let* ((pose-in-link (tf:transform-pose
                              *tf*
                              :pose current-pose-stamped
                              :target-frame target-link))
               (obj-msg-plain (create-collision-object-message
                               name pose-in-link
                               :primitive-shapes primitive-shapes
                               :mesh-shapes mesh-shapes
                               :plane-shapes plane-shapes))
               (obj-msg (roslisp:modify-message-copy
                         obj-msg-plain
                         operation (roslisp-msg-protocol:symbol-code
                                    'moveit_msgs-msg:collisionobject
                                    :add)))
               (attach-msg (roslisp:make-msg
                            "moveit_msgs/AttachedCollisionObject"
                            object (roslisp:modify-message-copy
                                    obj-msg-plain
                                    operation (roslisp-msg-protocol:symbol-code
                                               'moveit_msgs-msg:collisionobject
                                               :remove)
                                    id name)))
               (world-msg (roslisp:make-msg
                           "moveit_msgs/PlanningSceneWorld"
                           collision_objects (vector obj-msg)))
               (scene-msg (roslisp:make-msg
                           "moveit_msgs/PlanningScene"
                           world world-msg
                           (attached_collision_objects robot_state) (vector
                                                                     attach-msg)
                           is_diff t)))
          (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
            (roslisp:ros-info
             (moveit)
             "Detaching collision object `~a' from link `~a'."
             name (tf:frame-id current-pose-stamped))))))))
