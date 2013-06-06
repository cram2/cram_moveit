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

(defvar *move-group-action-client* nil
  "Action client for the MoveGroup action.")

(defun init-moveit-bridge ()
  "Sets up the basic action client communication handles for the
MoveIt! framework."
  (setf *move-group-action-client*
        (actionlib:make-action-client
         "move_group" "moveit_msgs/MoveGroupAction")))

(register-ros-init-function init-moveit-bridge)

(defun move-link-pose (link-name planning-group pose-stamped
                       &key allowed-collision-objects)
  "Calls the MoveIt! MoveGroup action. The link identified by
`link-name' is tried to be positioned in the pose given by
`pose-stamped'. Returns multiply bound values denoting boolean
success, and the numerical return value as defined in
moveit_msgs/MoveItErrorCodes."
  (declare (ignore allowed-collision-objects)) ;; Implement this.
  (let ((mpreq (make-message
                "moveit_msgs/MotionPlanRequest"
                :group_name planning-group
                :num_planning_attempts 1
                :allowed_planning_time 5.0
                :goal_constraints
                (vector
                 (make-message
                  "moveit_msgs/Constraints"
                  :position_constraints
                  (vector
                   (make-message
                    "moveit_msgs/PositionConstraint"
                    :weight 1.0
                    :link_name link-name
                    :header
                    (make-message
                     "std_msgs/Header"
                     :frame_id (tf:frame-id pose-stamped)
                     :stamp (tf:stamp pose-stamped))
                    :constraint_region
                    (make-message
                     "moveit_msgs/BoundingVolume"
                     :primitives
                     (vector
                      (make-message
                       "shape_msgs/SolidPrimitive"
                       :type 1 ;; BOX, find out how to read symbol
                       :dimensions (vector 0.01 0.01 0.01)))
                     :primitive_poses
                     (vector
                      (tf:pose->msg pose-stamped)))))
                  :orientation_constraints
                  (vector
                   (make-message
                    "moveit_msgs/OrientationConstraint"
                    :weight 1.0
                    :link_name link-name
                    :header
                    (make-message
                     "std_msgs/Header"
                     :frame_id (tf:frame-id pose-stamped)
                     :stamp (tf:stamp pose-stamped))
                    :orientation
                    (make-message
                     "geometry_msgs/Quaternion"
                     :x (tf:x (tf:orientation pose-stamped))
                     :y (tf:y (tf:orientation pose-stamped))
                     :z (tf:z (tf:orientation pose-stamped))
                     :w (tf:w (tf:orientation pose-stamped)))
                    :absolute_x_axis_tolerance 0.01
                    :absolute_y_axis_tolerance 0.01
                    :absolute_z_axis_tolerance 0.01)))))))
    (let ((result (actionlib:call-goal
                   *move-group-action-client*
                   (actionlib:make-action-goal *move-group-action-client*
                     :request mpreq))))
      (roslisp:with-fields (error_code) result
        (roslisp:with-fields (val) error_code
          (unless (eql val 1)
            (roslisp:ros-warn
             (moveit) "MoveGroup failed. Code: ~a~%" error_code))
          (values (eql val 1) val))))))
