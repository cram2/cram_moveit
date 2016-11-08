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
;;;     * Neither the name of the Universitaet Bremen nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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
   (pose :initform nil
         :initarg :pose
         :reader :pose)
   (color :initform nil
          :initarg :color
          :reader :color)
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

(defun 3dvector->vector (3dvector)
  (vector (tf:x 3dvector) (tf:y 3dvector) (tf:z 3dvector)))

(defgeneric register-collision-object (object &rest rest))

(defmethod register-collision-object ((object object-designator)
                                      &key pose-stamped add)
  (flet ((primitive-code (symbol)
           (let ((symbol (or symbol :box)))
             (roslisp-msg-protocol:symbol-code
              'shape_msgs-msg:solidprimitive symbol))))
    (let* ((name (string-upcase (string (desig-prop-value object :name))))
           (shape-prop (or (desig-prop-value object :shape) :box))
           (shape (primitive-code shape-prop))
	   (dim-prop (desig-prop-value object :dimensions))
           (dimensions (or
			(when dim-prop
			  (cond ((string= (symbol-name (class-name dim-prop)) "3D-VECTOR")
				 (cond ((eql shape-prop :cylinder)
					(vector (tf:z dim-prop) (tf:x dim-prop)))
				       (t (3dvector->vector dim-prop))))
				((vectorp dim-prop) dim-prop)))
                        (case shape-prop
                          (:box (vector 0.1 0.1 0.1))
                          (:sphere (vector 0.1 0.1))
                          (:cylinder (vector 0.03 0.2))
                          (:round (vector 0.2 0.08))
                          (:cone (vector 0.1 0.1)))))
           (pose-stamped
             (or pose-stamped
                 (when (desig-prop-value object :at)
                   (reference (desig-prop-value object :at))))))
      (unless pose-stamped
        (roslisp:ros-warn (moveit) "No pose-stamped given (neither manually nor in the object-designator) when adding object-designator ~a to the collision environment." object)
        (when add
          (roslisp:ros-error (moveit) "You chose to `add' the object to the scene directly without setting a pose. This will result in unexpected behavior at best. Blindly following your directive anyway.")))
      (register-collision-object
       name
       :primitive-shapes (list (roslisp:make-msg
                                "shape_msgs/SolidPrimitive"
                                type shape
                                dimensions
                                (cond ((eql shape-prop :round)
                                       (vector (elt dimensions 2)
                                               (/ (elt dimensions 1) 2)))
                                      (t dimensions))))
       :pose-stamped pose-stamped
       :color (desig-prop-value object :color))
      (when add
        (add-collision-object name pose-stamped t))
      name)))

(defmethod register-collision-object ((name string)
                                      &key
                                        primitive-shapes
                                        mesh-shapes
                                        plane-shapes
                                        pose-stamped
                                        color)
  (let* ((name (string-upcase (string name)))
         (obj (or (let ((obj (named-collision-object name)))
                    (when obj
                      (setf (slot-value obj 'primitive-shapes) primitive-shapes))) 
                  (let ((obj-create
                          (make-instance 'collision-object
                            :name name
                            :primitive-shapes primitive-shapes
                            :mesh-shapes mesh-shapes
                            :plane-shapes plane-shapes
                            :color color)))
                    (push obj-create *known-collision-objects*)
                    obj-create))))
    (when (and obj pose-stamped)
      (set-collision-object-pose name pose-stamped))))

(defun unregister-collision-object (name)
  (let ((name (string-upcase (string name))))
    (setf *known-collision-objects*
          (remove name *known-collision-objects*
                  :test (lambda (name object)
                          (equal name (slot-value object 'name)))))))

(defun named-collision-object (name)
  (let* ((name (string-upcase (string name)))
         (position (position name *known-collision-objects*
                             :test (lambda (name object)
                                     (string= name (slot-value object 'name))))))
    (when position
      (nth position *known-collision-objects*))))

(defun collision-object-pose (name)
  (let* ((col-obj (named-collision-object name)))
    (when col-obj
      (slot-value col-obj 'pose))))

(defun set-collision-object-pose (name pose-stamped)
  (let* ((col-obj (named-collision-object name)))
    (when col-obj
      (setf (slot-value col-obj 'pose) pose-stamped))))

(defun create-collision-object-message (name pose-stamped 
                                        &key
                                          primitive-shapes
                                          mesh-shapes
                                          plane-shapes)
  (let* ((name (string name)))
    (unless (or primitive-shapes mesh-shapes plane-shapes)
      (cpl:fail 'no-collision-shapes-defined))
    (flet* ((resolve-pose (pose-msg)
                          (or pose-msg (to-msg (cl-transforms:make-pose (cl-transforms:origin pose-stamped) (cl-transforms:orientation pose-stamped)))))
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
                       (stamp header) (stamp pose-stamped)
                       (frame_id header) (frame-id pose-stamped)
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
        obj-msg))))

(defun make-object-color (id color)
  (let ((col-vec (case (intern (string-upcase (string color)) :keyword)
                   (:blue (vector 0.0 0.0 1.0))
                   (:red (vector 1.0 0.0 0.0))
                   (:green (vector 0.0 1.0 0.0))
                   (:yellow (vector 1.0 1.0 0.0))
                   (:black (vector 0.0 0.0 0.0))
                   (:white (vector 1.0 1.0 1.0))
                   (t (vector 1.0 0.0 1.0)))))
    (roslisp:make-message
     "moveit_msgs/ObjectColor"
     id id
     (r color) (elt col-vec 0)
     (g color) (elt col-vec 1)
     (b color) (elt col-vec 2)
     (a color) 1.0)))


(defun republish-collision-environment ()
  (loop for object in *known-collision-objects*
        do (add-collision-object (slot-value object 'name) nil t)))

(defun height-of-object (collision-object)
  ;; NOTE(winkler): Just using the first one for now.
  (let ((primitive-shape (first (slot-value collision-object
                                            'primitive-shapes))))
    (or (when primitive-shape
          (with-fields (type dimensions) primitive-shape
            (ecase type
              (1 (elt dimensions 2)) ;; Box
              (3 (elt dimensions 0))))) ;; Cylinder
        0.0)))
    

(defun add-collision-object (name &optional pose-stamped quiet)
  (when name
    (let* ((name (string-upcase (string name)))
           (col-obj (named-collision-object name)))
      (when col-obj
        (let ((pose-stamped (or pose-stamped
                                (slot-value col-obj 'pose))))
          (when pose-stamped
            (setf (slot-value col-obj 'pose) pose-stamped)
            (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
                  (mesh-shapes (slot-value col-obj 'mesh-shapes))
                  (plane-shapes (slot-value col-obj 'plane-shapes))
                  (color (slot-value col-obj 'color)))
              (declare (ignorable color))
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
                                 ;;object_colors (vector (make-object-color name color))
                                 is_diff t)))
                (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
                  (unless quiet
                    (roslisp:ros-info (moveit) "Added `~a' to environment server." name))
                  (publish-object-colors))))))))))

(defun remove-collision-object (name)
  (let* ((name (string-upcase (string name)))
         (col-obj (named-collision-object name)))
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
           "Removed `~a' from environment server." name))))))

(defun clear-all-moveit-collision-objects ()
  (let* ((obj-msg (roslisp:make-msg
                   "moveit_msgs/CollisionObject"
                   id ""
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
       "Removed every collision object from environment server."))))

(defmacro without-collision-objects (object-names &body body)
  `(unwind-protect
        (progn
          (dolist (object-name ,object-names)
            (remove-collision-object object-name))
          ,@body)
     (dolist (object-name ,object-names)
       (add-collision-object object-name))))

(defmacro without-collision-object (object-name &body body)
  `(without-collision-objects (list ,object-name) ,@body))

(defun clear-collision-objects ()
  (loop for col-obj in *known-collision-objects*
        do (remove-collision-object (slot-value col-obj 'name))))

(defun clear-collision-environment ()
  (clear-collision-objects)
  (setf *known-collision-objects* nil)
  (roslisp:ros-info (moveit) "Cleared collision environment."))

(defun attach-collision-object-to-link (name target-link
                                        &key current-pose-stamped touch-links)
  (let* ((name (string-upcase (string name)))
         (col-obj (named-collision-object name))
         (current-pose-stamped
           (tf:copy-pose-stamped
            (or current-pose-stamped
                (collision-object-pose name))
            :stamp 0.0)))
    (when (and col-obj current-pose-stamped)
      (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
            (mesh-shapes (slot-value col-obj 'mesh-shapes))
            (plane-shapes (slot-value col-obj 'plane-shapes)))
        (roslisp:ros-info (moveit) "Transforming link from ~a into ~a"
                          (frame-id current-pose-stamped)
                          target-link)
        (let* ((pose-in-link
                 (cl-transforms-stamped:transform-pose-stamped
                  *transformer*
                  :pose current-pose-stamped
                  :target-frame target-link
                  :timeout *tf-default-timeout*))
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
          (roslisp:publish *planning-scene-publisher* scene-msg)
          (set-collision-object-pose name pose-in-link)
          (roslisp:ros-info
           (moveit)
           "Attached collision object `~a' to link `~a'."
           name target-link))))))

(defun detach-collision-object-from-link (name target-link
                                          &key current-pose-stamped)
  (let* ((name (string-upcase (string name)))
         (col-obj (named-collision-object name))
         (current-pose-stamped (or current-pose-stamped
                                   (collision-object-pose name))))
    (when (and col-obj current-pose-stamped)
      (let ((primitive-shapes (slot-value col-obj 'primitive-shapes))
            (mesh-shapes (slot-value col-obj 'mesh-shapes))
            (plane-shapes (slot-value col-obj 'plane-shapes))
            (time (roslisp:ros-time)))
        ;; (unless (cl-tf:wait-for-transform
        ;;          *tf*
        ;;          :timeout 5.0
        ;;          :time time
        ;;          :source-frame (frame-id current-pose-stamped)
        ;;          :target-frame target-link)
        ;;   (cpl:fail 'pose-not-transformable-into-link))
        (let* ((pose-in-link (cl-transforms-stamped:transform-pose-stamped
                              *transformer*
                              :pose (copy-pose-stamped
                                     current-pose-stamped
                                     :stamp time)
                              :target-frame target-link
                              :timeout *tf-default-timeout*))
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
          (roslisp:publish *planning-scene-publisher* scene-msg)
          (set-collision-object-pose name pose-in-link)
          (roslisp:ros-info
           (moveit)
           "Detaching collision object `~a' from link `~a'."
           name (frame-id current-pose-stamped)))))))

(defun detach-all-attachments ()
  (let* ((planning-scene
           (roslisp:call-service
            "/get_planning_scene"
            'moveit_msgs-srv:GetPlanningScene
            :components (roslisp:make-message
                         "moveit_msgs/PlanningSceneComponents"
                         (components) 4)))
         (attached-objects
           (with-fields (scene) planning-scene
             (with-fields (robot_state) scene
               (with-fields (attached_collision_objects) robot_state
                 (map 'list (lambda (attached-collision-object)
                              (with-fields (link_name object) attached-collision-object
                                (with-fields (id) object
                                  `(,link_name ,id))))
                      attached_collision_objects))))))
    (loop for attached-object in attached-objects
          do (destructuring-bind (link-name object-id) attached-object
               (detach-collision-object-from-link object-id link-name)))))

(defmethod cram-occasions-events::on-event object-perceived ((event cram-plan-occasions-events::object-perceived-event))
  (let ((desig (cram-plan-occasions-events::event-object-designator
                event)))
    (when desig
      (let* ((name (register-collision-object desig))
             (collision-object (named-collision-object name))
             (pose-stamped (collision-object-pose name))
             (pose-stamped
               (let ((height (height-of-object collision-object)))
                 (tf:copy-pose-stamped
                  pose-stamped
                  :origin (tf:v+ (tf:origin pose-stamped)
                                 (tf:make-3d-vector
                                  0 0 (/ height 2)))))))
        (add-collision-object
         name pose-stamped)))))
