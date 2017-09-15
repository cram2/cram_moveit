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

(defvar *moveit-pose-validity-check-lock* nil)
(defparameter *object-reference-frame* "map")
(defparameter *needs-ft-fix* nil)

(defun init-moveit-bridge ()
  "Sets up the basic action client communication handles for the
MoveIt! framework and registers known conditions."
  (register-known-moveit-errors)
  (setf *planning-scene-publisher*
        (roslisp:advertise
         "/planning_scene"
         "moveit_msgs/PlanningScene" :latch t))
  (setf *robot-state-display-publisher*
        (roslisp:advertise
         *robot-state-display-topic*
         "moveit_msgs/DisplayRobotState" :latch t))
  (setf *joint-states-fluent*
        (cram-language:make-fluent :name "joint-state-tracker" :allow-tracing nil))
  (setf *joint-states-subscriber*
        (roslisp:subscribe "/joint_states"
                           "sensor_msgs/JointState"
                           #'joint-states-callback))
  (connect-action-client)
  (setf *moveit-action-access-lock*
        (make-lock :name "moveit-action-access"))
  (setf *moveit-pose-validity-check-lock*
        (make-lock :name "validity-check-access")))

(roslisp-utilities:register-ros-init-function init-moveit-bridge)

(cut:define-hook cram-language::on-begin-motion-planning (link-name))
(cut:define-hook cram-language::on-finish-motion-planning (id))
(cut:define-hook cram-language::on-begin-motion-execution ())
(cut:define-hook cram-language::on-finish-motion-execution (id))

(defmethod cram-occasions-events:on-event
    ((event cram-plan-occasions-events:object-removed-event))
  (remove-collision-object (cram-plan-occasions-events:event-object-name event)))

(defmethod cram-occasions-events:on-event
    ((event cram-plan-occasions-events:object-perceived-event))
  (let ((object (cram-plan-occasions-events:event-object-designator event)))
    (register-collision-object
     object :add t
            :pose-stamped (cl-transforms-stamped:transform-pose-stamped
                           *transformer*
                           :pose (cl-transforms-stamped:ensure-pose-stamped
                                  (desig-prop-value
                                   (desig-prop-value object :at)
                                   :pose)
                                  *object-reference-frame* 0.0)
                           :target-frame *object-reference-frame*
                           :timeout *tf-default-timeout*))))

(defmethod cram-occasions-events:on-event
    ((event cram-plan-occasions-events:object-updated-event))
  (let* ((object (cram-plan-occasions-events:event-object-designator event)))
    (register-collision-object
     object :add t
            :pose-stamped
            (cl-transforms-stamped:transform-pose-stamped
                           *transformer*
                           :pose (desig-prop-value
                                  (desig-prop-value object :at)
                                  :pose)
                           :target-frame *object-reference-frame*
                           :timeout *tf-default-timeout*))))

(defun move-joints (planning-group joint-names joint-positions
                    &key (wait-for-execution t))
  (move-link-pose nil planning-group nil
                  :joint-names joint-names
                  :joint-positions joint-positions
                  :wait-for-execution wait-for-execution))

(defun get-end-robot-state (start-state trajectory)
  (let* ((joint-names (coerce (roslisp:msg-slot-value (roslisp:msg-slot-value start-state :joint_state) :name) 'list))
         (joint-velocities nil)
         (joint-values (coerce (roslisp:msg-slot-value (roslisp:msg-slot-value start-state :joint_state) :position) 'list))
         (mdof-joint-names (coerce (roslisp:msg-slot-value (roslisp:msg-slot-value start-state :multi_dof_joint_state) :joint_names) 'list))
         (mdof-joint-values (coerce (roslisp:msg-slot-value (roslisp:msg-slot-value start-state :multi_dof_joint_state) :transforms) 'list))
         (joint-alist (mapcar (lambda (name value)
                                (cons name value))
                              joint-names joint-values))
         (mdof-joint-alist (mapcar (lambda (name value)
                                     (cons name value))
                                   mdof-joint-names mdof-joint-values))
         (joint-trajectory (roslisp:msg-slot-value trajectory :joint_trajectory))
         (mdof-joint-trajectory (roslisp:msg-slot-value trajectory :multi_dof_joint_trajectory))
         (joint-trajectory-names (coerce (roslisp:msg-slot-value joint-trajectory :joint_names) 'list))
         (mdof-trajectory-names (coerce (roslisp:msg-slot-value mdof-joint-trajectory :joint_names) 'list))
         (joint-trajectory-values (coerce (roslisp:msg-slot-value joint-trajectory :points) 'list))
         (mdof-trajectory-values (coerce (roslisp:msg-slot-value mdof-joint-trajectory :points) 'list))
         (last-joint-value (car (last joint-trajectory-values)))
         (last-mdof-joint-value (car (last mdof-trajectory-values))))
    (loop for name in joint-trajectory-names
          for k from 0 upto (- (length joint-trajectory-names) 1) do
      (let* ((value (aref (roslisp:msg-slot-value last-joint-value :positions) k))
             (present (assoc name joint-alist :test #'equal)))
        (if present
          (rplacd (assoc name joint-alist :test #'equal)
                  value)
          (setf joint-alist
                (cons (cons name value) joint-alist)))))
    (setf joint-names (mapcar (lambda (arg)
                                (car arg))
                              joint-alist))
    (setf joint-velocities (mapcar (lambda (arg)
                                     (declare (ignore arg))
                                     0.0)
                                   joint-alist))
    (setf joint-values  (mapcar (lambda (arg)
                                  (cdr arg))
                                joint-alist))
    (loop for name in mdof-trajectory-names
          for k from 0 upto (- (length mdof-trajectory-names) 1) do
      (let* ((value (aref (roslisp:msg-slot-value last-mdof-joint-value :transforms) k))
             (present (assoc name mdof-joint-alist :test #'equal)))
        (if present
          (rplacd (assoc name mdof-joint-alist :test #'equal)
                  value)
          (cons (cons name value)
                mdof-joint-alist))))
    (setf mdof-joint-names (mapcar (lambda (arg)
                                     (car arg))
                                   mdof-joint-alist))
    (setf mdof-joint-values (mapcar (lambda (arg)
                                      (cdr arg))
                                    mdof-joint-alist))
    (roslisp:make-message "moveit_msgs/RobotState"
                          :attached_collision_objects (roslisp:msg-slot-value start-state :attached_collision_objects)
                          :is_diff (roslisp:msg-slot-value start-state :is_diff)
                          :joint_state (roslisp:make-message "sensor_msgs/JointState"
                                                             :effort (vector)
                                                             :header (roslisp:msg-slot-value (roslisp:msg-slot-value start-state :joint_state) :header)
                                                             :name (coerce joint-names 'vector)
                                                             :position (coerce joint-values 'vector)
                                                             :velocity (coerce joint-velocities 'vector))
                          :multi_dof_joint_state (roslisp:make-message "sensor_msgs/MultiDOFJointState"
                                                             :header (roslisp:msg-slot-value (roslisp:msg-slot-value start-state :multi_dof_joint_state) :header)
                                                             :joint_names (coerce mdof-joint-names 'vector)
                                                             :transforms (coerce mdof-joint-values 'vector)
                                                             :twist (vector)
                                                             :wrench (vector)))))

(defun move-link-pose (link-name planning-group pose-stamped
                       &key allowed-collision-objects
                         (path-constraints-msg (roslisp:make-msg "moveit_msgs/Constraints"))
                         plan-only touch-links
                         ignore-collisions
                         start-state
                         collidable-objects
                         joint-names joint-positions
                         (wait-for-execution t)
                         max-tilt
                         reference-frame
                         raise-elbow
                         additional-touch-link-groups
                         additional-collision-objects-groups
                         additional-values)
  "Calls the MoveIt! MoveGroup action. The link identified by
  `link-name' is tried to be positioned in the pose given by
  `pose-stamped'. Returns `T' on success and `nil' on failure, in
  which case a failure condition is signalled, based on the error code
  returned by the MoveIt! service (as defined in
  moveit_msgs/MoveItErrorCodes)."
  ;; NOTE(winkler): This is a PR2 specific fix for when the force
  ;; torque sensor is included in the URDF.
  (when *needs-ft-fix*
    (setf additional-touch-link-groups
          (append nil;additional-touch-link-groups
                  `(("l_gripper_palm_link")
                    ("l_force_torque_link"))))
    (setf additional-collision-objects-groups
          (append nil;additional-collision-objects-groups
                  `(("l_force_torque_link")
                    ("l_forearm_link"
                     "l_wrist_roll_link"
                     "l_gripper_motor_accelerometer_link"))))
    (setf additional-values
          (append nil;additional-values
                  `((t) (t)))))
  (format t "~a~%" additional-touch-link-groups)
  (format t "~a~%" additional-collision-objects-groups)
  (format t "~a~%" additional-values)
  (cond ((and joint-names joint-positions)
         (ros-info (moveit) "Move joints"))
        (t (ros-info (moveit)
                     "Move link: ~a (~a, ignore collisions: ~a, plan only: ~a, raise elbow: ~a)"
                     link-name planning-group ignore-collisions plan-only raise-elbow)))
  (let* ((log-id (first (cram-language::on-begin-motion-planning link-name)))
         (planning-results
           (unwind-protect
                (let* ((start-state (or start-state
                                        (make-message "moveit_msgs/RobotState"
                                                      :is_diff t)))
                       (allowed-collision-objects
                         (mapcar
                          #'string
                          (cond (ignore-collisions
                                 (loop for obj in *known-collision-objects*
                                       collect (slot-value obj 'name)))
                                (t allowed-collision-objects))))
                       (touch-links
                         (mapcar (lambda (x) (string x)) touch-links))
                       (link-names (cond ((listp link-name) link-name)
                                         (t `(,link-name))))
                       (poses-stamped (mapcar
                                       (lambda (pose-stamped)
                                         (pose->pose-stamped
                                          (cl-transforms-stamped:unslash-frame
                                           (frame-id pose-stamped))
                                          (stamp pose-stamped)
                                          pose-stamped))
                                       (cond ((listp pose-stamped) pose-stamped)
                                             (t `(,pose-stamped)))))
                       (max-tilts (cond ((listp max-tilt) max-tilt)
                                        (t `(,max-tilt))))
                       (reference-frames
                         (cond (reference-frame
                                (cond ((listp reference-frame)
                                       reference-frame)
                                      (t `(,reference-frame))))
                               (poses-stamped
                                (list (frame-id (car poses-stamped)))))))
                  (let* ((mpreq (make-message
                                 "moveit_msgs/MotionPlanRequest"
                                 :group_name planning-group
                                 :num_planning_attempts 3
                                 :allowed_planning_time 30.0
                                 :start_state start-state
                                 ;; :trajectory_constraints
                                 ;; (make-trajectory-constraints
                                 ;;  :link-names link-names
                                 ;;  :reference-frames reference-frames
                                 ;;  :max-tilts max-tilts
                                 ;;  :reference-orientations
                                 ;;  (mapcar (lambda (pose)
                                 ;;            (cl-transforms:orientation pose))
                                 ;;          poses-stamped))
                                 :goal_constraints
                                 (map 'vector #'identity
                                      (append
                                       (map 'list #'identity
                                            (make-pose-goal-constraints
                                             link-names poses-stamped
                                             joint-names joint-positions
                                             :raise-elbow raise-elbow))))
                                 ;; (cond ((and joint-names joint-positions)
                                 ;;        (make-joint-goal-constraints
                                 ;;         joint-names joint-positions))
                                 ;;       (t (make-pose-goal-constraints
                                 ;;           link-names poses-stamped)))
                                 ))
                         (options
                           (make-message
                            "moveit_msgs/PlanningOptions"
                            :planning_scene_diff
                            (make-message
                             "moveit_msgs/PlanningScene"
                             :is_diff t
                             :allowed_collision_matrix
                             (relative-collision-matrix-msg
                              (append
                               `(,touch-links
                                 ,collidable-objects)
                               additional-touch-link-groups)
                              (append
                               `(,allowed-collision-objects
                                 ,(when collidable-objects
                                    (loop for obj in *known-collision-objects*
                                          collect (slot-value obj 'name))))
                               additional-collision-objects-groups)
                              (append
                               `(t t)
                               additional-values))
                             :robot_state start-state)
                            :plan_only t;plan-only
                            :replan t
                            :replan_attempts 3)))
                    (cpl:with-failure-handling
                        ((invalid-motion-plan (f)
                           (declare (ignore f))
                           (ros-warn (moveit) "Invalid motion plan. Rethrowing as failed manipulation attempt.")
                           (error 'manipulation-failed)))
                      (roslisp:with-fields (error_code
                                            trajectory_start
                                            planned_trajectory)
                          (send-action *move-group-action-client*
                                       :request mpreq
                                       :planning_options options)
                        (roslisp:with-fields (val) error_code
                          (signal-moveit-error val))
                        (list trajectory_start planned_trajectory)))))
             (cram-language::on-finish-motion-planning log-id))))
    (cond ((not plan-only)
           (let ((log-id (first (cram-language::on-begin-motion-execution))))
             (unwind-protect
                  (execute-trajectories
                   `(,(second planning-results))
                   :wait-for-execution wait-for-execution)
               (cram-language::on-finish-motion-execution log-id)))))
    (ros-info (moveit) "Done moving")
    (values (first planning-results) (second planning-results))))

(defmacro move-multiple-links-independently
    (link-names planning-groups poses-stamped ignore-collisions)
  (let ((trajectories
          (mapcar (lambda (link-name
                           planning-group
                           pose-stamped
                           ignore-collision)
                    (multiple-value-bind (start-state
                                          trajectory)
                        (move-link-pose
                         link-name planning-group pose-stamped
                         :ignore-collisions ignore-collision
                         :plan-only t)
                      (declare (ignore start-state))
                      trajectory))
                  link-names planning-groups poses-stamped
                  ignore-collisions)))
    (execute-trajectories trajectories)))

(defun plan-base-movement (x y theta)
  (move-link-joint-states
   "base"
   (list (cons "virtual_joint/theta" theta)
         (cons "virtual_joint/x" x)
         (cons "virtual_joint/y" y))))

(defun move-link-joint-states (planning-group joint-states)
  (let* ((mpreq (make-message
                 "moveit_msgs/MotionPlanRequest"
                 :group_name planning-group
                 :num_planning_attempts 1
                 :allowed_planning_time 1
                 :goal_constraints
                 (vector
                  (make-message
                   "moveit_msgs/Constraints"
                   :joint_constraints
                   (map 'vector (lambda (joint-state)
                                  (make-message
                                   "moveit_msgs/JointConstraint"
                                   :joint_name (car joint-state)
                                   :position (cdr joint-state)
                                   :tolerance_below 0.1
                                   :tolerance_above 0.1
                                   :weight 1.0))
                        joint-states)))))
         (options
           (make-message
            "moveit_msgs/PlanningOptions"
            :planning_scene_diff
            (make-message
             "moveit_msgs/PlanningScene"
             :is_diff t)
            :plan_only t)))
    (cpl:with-failure-handling
        ((invalid-motion-plan (f)
           (declare (ignore f))
           (ros-warn (moveit) "Invalid motion plan. Rethrowing.")
           (error 'manipulation-failed)))
      (let ((result (send-action *move-group-action-client*
                                 :request mpreq
                                 :planning_options options)))
        (cond (result
               (roslisp:with-fields (error_code
                                     trajectory_start
                                     planned_trajectory)
                   result
                 (roslisp:with-fields (val) error_code
                   (unless
                       (eql val
                            (roslisp-msg-protocol:symbol-code
                             'moveit_msgs-msg:moveiterrorcodes
                             :success))
                     (signal-moveit-error val))
                   (values
                    trajectory_start planned_trajectory))))
              (t (ros-error (moveit)
                            "Empty actionlib response.")
                 (sleep 5) ;; Sleep here to give MoveIt! time to come up again.
                 (connect-action-client)
                 (error 'planning-failed)))))))

(defun execute-trajectory (trajectory &key (wait-for-execution t))
  (let ((result (call-service "/execute_kinematic_path"
                              'moveit_msgs-srv:ExecuteKnownTrajectory
                              :trajectory trajectory
                              :wait_for_execution wait-for-execution)))
    (roslisp:with-fields (error_code) result
      (roslisp:with-fields (val) error_code
        (unless (eql val (roslisp-msg-protocol:symbol-code
                          'moveit_msgs-msg:moveiterrorcodes
                          :success))
          (signal-moveit-error val))))
    t))

(defun execute-trajectories (trajectories &key (wait-for-execution t)
                                            ignore-va)
  (ros-info (moveit) "Executing ~a trajector~a."
            (length trajectories)
            (if (= (length trajectories) 1) "y" "ies"))
  (let ((trajectories (cpl:mapcar-clean #'identity trajectories)))
    (when trajectories
      (execute-trajectory
       (merge-trajectories trajectories :ignore-va ignore-va)
       :wait-for-execution wait-for-execution))))

(defun trajectory-length (trajectory)
  (with-fields (joint_trajectory) trajectory
    (with-fields (points) joint_trajectory
      (length points))))

(defun longest-trajectory (trajectories)
  (loop for trajectory in trajectories
        maximizing (trajectory-length trajectory)))

(defun trajectory-names (trajectory)
  (with-fields (joint_trajectory) trajectory
    (with-fields (joint_names) joint_trajectory
      joint_names)))

(defun merge-trajectory-names (trajectories)
  (loop for trajectory in trajectories
        for names = (map 'list #'identity (trajectory-names trajectory))
        append names into merged-names
        finally (return merged-names)))

(defun merged-trajectory-positions (trajectories index)
  (map 'vector #'identity
       (loop for trajectory in trajectories
             for positions = (map 'list
                                  #'identity
                                  (with-fields (joint_trajectory) trajectory
                                    (with-fields (points) joint_trajectory
                                      (with-fields (positions) (elt points index)
                                        positions))))
             append positions into merged
             finally (return merged))))

(defun merged-trajectory-velocities (trajectories index)
  (map 'vector #'identity
       (loop for trajectory in trajectories
             for velocities = (map 'list
                                   #'identity
                                   (with-fields (joint_trajectory) trajectory
                                     (with-fields (points) joint_trajectory
                                       (with-fields (velocities) (elt points index)
                                         velocities))))
             append velocities into merged
             finally (return merged))))

(defun merged-trajectory-accelerations (trajectories index)
  (map 'vector #'identity
       (loop for trajectory in trajectories
             for accelerations = (map 'list
                                      #'identity
                                      (with-fields (joint_trajectory) trajectory
                                        (with-fields (points) joint_trajectory
                                          (with-fields (accelerations) (elt points index)
                                            accelerations))))
             append accelerations into merged
             finally (return merged))))

(defun latest-time-for-trajectory-point (trajectories index)
  (loop for trajectory in trajectories
        maximizing (with-fields (joint_trajectory) trajectory
                     (with-fields (points) joint_trajectory
                       (with-fields (time_from_start) (elt points index)
                         time_from_start)))
        into max-val
        finally (return max-val)))

(defun latest-time-for-trajectory (trajectory)
  (with-fields (joint_trajectory) trajectory
    (with-fields (points) joint_trajectory
      (loop for point in (map 'list #'identity points)
            maximizing (with-fields (time_from_start) point
                         time_from_start)
            into max-val
            finally (return max-val)))))

(defun latest-time-for-trajectories (trajectories)
  (loop for trajectory in trajectories
        maximizing (latest-time-for-trajectory trajectory)
        into max-val
        finally (return max-val)))

(defun stretch-trajectories (trajectories stretch-to-length)
  (let ((longest-trajectory nil))
    (loop for trajectory in trajectories
          when (= (trajectory-length trajectory) stretch-to-length)
            do (setf longest-trajectory trajectory))
    (labels ((longest-trajectory-point-at-index (i)
               (with-fields (joint_trajectory) longest-trajectory
                 (with-fields (points) joint_trajectory
                   (elt points i)))))
      (mapcar (lambda (trajectory)
                (let ((len (trajectory-length trajectory)))
                  (with-fields (joint_trajectory
                                multi_dof_joint_trajectory) trajectory
                    (with-fields (points) joint_trajectory
                      (with-fields (header joint_names) joint_trajectory
                        (make-message
                         "moveit_msgs/RobotTrajectory"
                         :joint_trajectory
                         (make-message
                          "trajectory_msgs/JointTrajectory"
                          :header header
                          :joint_names joint_names
                          :points
                          (map
                           'vector #'identity
                           (append
                            (mapcar (lambda (point i)
                                      (roslisp:modify-message-copy
                                       point
                                       time_from_start
                                       (with-fields (time_from_start)
                                           (longest-trajectory-point-at-index i)
                                         time_from_start)))
                                    (map 'list #'identity points)
                                    (loop for i from 0 below len
                                          collect i))
                            (mapcar (lambda (i)
                                      (roslisp:modify-message-copy
                                       (elt points (1- (length points)))
                                       velocities (vector)
                                       accelerations (vector)
                                       time_from_start
                                       (with-fields (time_from_start)
                                           (longest-trajectory-point-at-index i)
                                         time_from_start)))
                                    (loop for i from len below stretch-to-length
                                          collect i)))))
                         :multi_dof_joint_trajectory
                         multi_dof_joint_trajectory))))))
              trajectories))))

(defun merge-trajectories (trajectories &key ignore-va)
  (let* ((longest (longest-trajectory trajectories))
         (merged-trajectory-names
           (merge-trajectory-names trajectories))
         (stretched-trajectories
           (stretch-trajectories
            trajectories longest)))
    (make-message
     "moveit_msgs/RobotTrajectory"
     :joint_trajectory
     (make-message
      "trajectory_msgs/JointTrajectory"
      :joint_names (map 'vector #'identity merged-trajectory-names)
      :points
      (map 'vector #'identity
           (loop for i from 0 below longest
                 collect
                 (make-message
                  "trajectory_msgs/JointTrajectoryPoint"
                  :positions (merged-trajectory-positions
                              stretched-trajectories i)
                  :velocities
                  (cond (ignore-va (vector))
                        (t (merged-trajectory-velocities
                            stretched-trajectories i)))
                  :accelerations 
                  (cond (ignore-va (vector))
                        (t (merged-trajectory-accelerations
                            stretched-trajectories i)))
                  :time_from_start (+ (latest-time-for-trajectory-point
                                       stretched-trajectories i)
                                      1.0))))))))

(defun concatenate-trajectories (trajectories &key ignore-va)
  "Concatenates multiple trajectories into a single one, taking care
of timing of individual points. If `ignore-va' is set, the velocity
and acceleration fields in the final trajectory are left blank,
leaving their values to the executing controller."
  ;; TODO(winkler): The resulting trajectory it not *quite*
  ;; correct. Somehow, it is a bit jiggly when executing it on the
  ;; controller. This needs fixing.
  (let ((time-mult 1.0))
    (make-message
     "moveit_msgs/RobotTrajectory"
     :joint_trajectory
     (make-message
      "trajectory_msgs/JointTrajectory"
      :joint_names (map 'vector #'identity (trajectory-names (first trajectories)))
      :points
      (map 'vector #'identity
           (loop for trajectory in (map 'list #'identity trajectories)
                 with start-time = 0
                 with last-diff = 0
                 append
                 (with-fields (joint_trajectory) trajectory
                   (with-fields (points) joint_trajectory
                     (loop for point in (map 'list #'identity points)
                           with last-time = 0
                           collect
                           (with-fields (positions velocities accelerations time_from_start)
                               point
                             (setf last-diff (- time_from_start last-time))
                             (setf last-time time_from_start)
                             (make-message
                              "trajectory_msgs/JointTrajectoryPoint"
                              :positions positions
                              :velocities
                              (cond (ignore-va (vector))
                                    (t velocities))
                              :accelerations
                              (cond (ignore-va (vector))
                                    (t accelerations))
                              :time_from_start (* (+ time_from_start start-time) time-mult)))
                           finally (incf start-time (+ last-time last-diff)))))))))))

(defun compute-fk (link-names &key robot-state)
  "Computes the pose of named links, given robot-state. Will return
a list of (name pose-stamped) pairs, in which name is a string and 
pose-stamped is a cl-tf-datatypes:pose-stamped.

Parameters:

- link-names: a list of names
- robot-state: a RobotState message.

;(Invokes the move_group service /compute_fk)"
  (let* ((names (make-array (length link-names) :initial-contents link-names))
         (result (roslisp:call-service
                  "/compute_fk"
                  "moveit_msgs/GetPositionFK"
                  :fk_link_names names
                  :robot_state (or robot-state
                                   (make-message "moveit_msgs/RobotState")))))
       (roslisp:with-fields ((pose-stamped-vector pose_stamped)
                             (name-vector fk_link_names))
           result
         (mapcar (lambda (a b) (list a b))
                 (coerce name-vector 'list)
                 (coerce pose-stamped-vector 'list)))))

(defun compute-ik (link-name planning-group pose-stamped &key robot-state (avoid-collisions T))
  "Computes an inverse kinematics solution (if possible) of the given
kinematics goal (given the link name `link-name' to position, the
`planning-group' to take into consideration, and the final goal pose
`pose-stamped' for the given link). Returns the final joint state on
success, and `nil' otherwise."
  (let ((result (roslisp:call-service
                 "/compute_ik"
                 "moveit_msgs/GetPositionIK"
                 :ik_request
                 (make-message
                  "moveit_msgs/PositionIKRequest"
                  :group_name planning-group
                  :ik_link_names (vector link-name)
                  :avoid_collisions (if avoid-collisions T nil)
                  :pose_stamped_vector (vector (to-msg pose-stamped))
                  :robot_state (or robot-state
                                   (make-message "moveit_msgs/RobotState"))))))
    (roslisp:with-fields (solution error_code) result
      (roslisp:with-fields (val) error_code
        (unless (eql val (roslisp-msg-protocol:symbol-code
                          'moveit_msgs-msg:moveiterrorcodes
                          :success))
          (signal-moveit-error val))
        solution))))


(defun check-state-validity (robot-state-msg planning-group-name constraints-msg)
  "Uses the MoveIt! check_state_validity service to verify a given robot state.

Parameters:

:robot-state robot-state-msg is the state to verify
:group_name group-name is the planning group to use in collision checking.
:constraints constraints-msg defines other kinematic constraints on the state.

Returns a list of lists: ((\"state validity\" valid) (\"contacts\" contacts)
 (\"cost sources\" cost-sources) (\"constraint check result\" constraint-result))."
  (let* ((result (roslisp:call-service "/check_state_validity"
                                       "moveit_msgs/GetStateValidity"
                                       :robot_state robot-state-msg
                                       :group_name planning-group-name
                                       :constraints constraints-msg)))
    (roslisp:with-fields (valid contacts cost_sources constraint_result) result
      (list (list "state validity" valid)
            (list "contacts" contacts)
            (list "cost sources" cost_sources)
            (list "constraint check result" constraint_result)))))

(defun compute-cartesian-path (frame-name
                               robot-state-msg
                               group-name
                               link-name
                               waypoint-poses
                               max-step
                               jump-threshold
                               avoid-collisions
                               &key (path-constraints-msg
                                     (roslisp:make-msg "moveit_msgs/Constraints")))
  "Calls MoveIt's compute_cartesian_path to get a path from robot-state-msg
that passes through the waypoints given by waypoint-poses with the link link-name.
The waypoint poses are given in the frame identified by frame-name.
Between two waypoints, the generated path will have the link move in a line.
max-step gives the largest distance (in Cartesian space)
that will be allowed between two points in the generated path.
jump-threshold gives a distance in configuration (joint) space which,
if exceeded between consecutive waypoints, is interpreted as a jump in the IK solution.
If a jump happens, MoveIt! considers the path generation failed,
and will return a path that ends just before the jump (see \"completion fraction\" in the
return values)
avoid-collisions determines whether obstacle collisions
 (apart from those in the allowed collision matrix) are tolerated.
Set to T to signal collisions are not ok.
path-constraints-msg describes any other kinematic constraints the path must satisfy.

Return values:
is a list of lists: ((\"start state\" start_state)
                     (\"trajectory\" solution)
                     (\"completion fraction\" fraction))
fraction is a number between 0 and 1 which gives how much of the requested path
was actually produced. If a path satisfying the constraints and not
colliding with unwanted obstacles is found, then the completion fraction is 1.
A lesser number signals a kinematic jump or an obstacle collision."

    (let* ((result (roslisp:call-service "/compute_cartesian_path"
                                         "moveit_msgs/GetCartesianPath"
                                         :header (make-message "std_msgs/Header"
                                                               :stamp 0
                                                               :frame_id frame-name)
                                         :start_state robot-state-msg
                                         :group_name group-name
                                         :link_name link-name
                                         :waypoints waypoint-poses
                                         :max_step max-step
                                         :jump_threshold jump-threshold
                                         :avoid_collisions avoid-collisions
                                         :path_constraints path-constraints-msg)))
    (roslisp:with-fields (start_state solution fraction (val (val error_code))) result
      (unless (eql val (roslisp-msg-protocol:symbol-code
                        'moveit_msgs-msg:moveiterrorcodes :success))
              (signal-moveit-error val))
      (list (list "start state" start_state)
            (list "trajectory" solution)
            (list "completion fraction" fraction)))))

(defun plan-link-movement (link-name planning-group pose-stamped
                           &key allowed-collision-objects
                             start-robot-state
                             (path-constraints-msg (roslisp:make-msg "moveit_msgs/Constraints"))
                             touch-links
                             ignore-collisions
                             destination-validity-only
                             highlight-links
                             max-tilt)
  "Plans the movement of link `link-name' to given goal-pose
`pose-stamped', taking the planning group `planning-group' into
consideration. Returns the proposed trajectory, and final joint state
on finding a valid motion plan for the given configuration from the
current configuration. If the flag `destination-validity-only' is set,
only the final state (but not the motion path trajectory in between)
is returned. Setting this flag also speeds up the process very much,
as only the final configuration IK is generated."
  (cpl:with-failure-handling
      ((moveit:no-ik-solution (f)
         (declare (ignore f))
         (return))
       (moveit:planning-failed (f)
         (declare (ignore f))
         (return))
       (moveit:goal-violates-path-constraints (f)
         (declare (ignore f))
         (return))
       (moveit:invalid-goal-constraints (f)
         (declare (ignore f))
         (return))
       (moveit:invalid-motion-plan (f)
         (declare (ignore f))
         (return))
       (moveit:goal-in-collision (f)
         (declare (ignore f))
         (return)))
    (cond (destination-validity-only
           (let ((ik (compute-ik link-name planning-group pose-stamped)))
             (when (and ik highlight-links)
               (display-robot-state ik :highlight highlight-links))
             ik))
          (t (moveit:move-link-pose
              link-name
              planning-group pose-stamped
              :allowed-collision-objects allowed-collision-objects
              :plan-only t
              :path-constraints-msg path-constraints-msg
              :start-state start-robot-state
              :touch-links touch-links
              :ignore-collisions ignore-collisions
              :max-tilt max-tilt)))))


(defun plan-link-movements (link-name planning-group poses-stamped
                            &key allowed-collision-objects
                              path-constraints-msgs
                              touch-links default-collision-entries
                              ignore-collisions
                              destination-validity-only
                              highlight-links
                              start-robot-state
                              max-tilt)
"Compute plans for link-name from planning-group to reach the poses
in pose-stamped. Paths can be constrained via path-constraints-msgs
but if these are used, then the length of the path-constraints-msgs
list and poses-stamped must be the same."
  (declare (ignore default-collision-entries))
  (if path-constraints-msgs
    (if (and (listp path-constraints-msgs) (eql (length path-constraints-msgs) (length poses-stamped)))
      (every (lambda (pose-stamped path-constraints-msg)
               (plan-link-movement
                link-name planning-group pose-stamped
                :allowed-collision-objects allowed-collision-objects
                :touch-links touch-links
                :highlight-links highlight-links
                :path-constraints-msg path-constraints-msg
                :start-robot-state start-robot-state
                :ignore-collisions ignore-collisions
                :destination-validity-only destination-validity-only
                :max-tilt max-tilt))
           poses-stamped path-constraints-msgs)
      (progn (ros-warn (moveit) "cram-moveit received a call to plan-link-movements
where path-constraints-msgs was non nil and had a different length than poses-stamped.
Cannot match path constraints to poses.")
             (error 'planning-failed)))
    (every (lambda (pose-stamped)
             (plan-link-movement
              link-name planning-group pose-stamped
              :allowed-collision-objects allowed-collision-objects
              :touch-links touch-links
              :start-robot-state start-robot-state
              :ignore-collisions ignore-collisions
              :destination-validity-only destination-validity-only
              :max-tilt max-tilt))
           poses-stamped)))


(defun make-joint-goal-constraints (names positions)
  (vector
   (make-message
    "moveit_msgs/Constraints"
    :joint_constraints
    (map 'vector
         (lambda (name position)
           (make-message
            "moveit_msgs/JointConstraint"
            :joint_name name
            :position position
            :tolerance_above 0.01
            :tolerance_below 0.01
            :weight 1.0))
         names positions))))

(defun make-pose-goal-constraints (link-names poses-stamped names positions
                                   &key (tolerance-radius 0.01) raise-elbow)
  (let ((joint_constraints
          (map 'vector #'identity
               (append
                (mapcar
                 (lambda (name position)
                   (make-message
                    "moveit_msgs/JointConstraint"
                    :joint_name name
                    :position position
                    :tolerance_above 0.01
                    :tolerance_below 0.01
                    :weight 1.0))
                 names positions)
                (when (or (eql raise-elbow :left) (eql raise-elbow :right))
                  (list
                   (case raise-elbow
                     (:left
                      (make-message
                       "moveit_msgs/JointConstraint"
                       :joint_name "l_shoulder_lift_joint"
                       :position -0.25
                       :tolerance_above 0.25
                       :tolerance_below 0.25
                       :weight 1.0))
                     (:right
                      (make-message
                       "moveit_msgs/JointConstraint"
                       :joint_name "r_shoulder_lift_joint"
                       :position -0.25
                       :tolerance_above 0.25
                       :tolerance_below 0.25
                       :weight 1.0)))))))))
    (map 'vector
         (lambda (link-name pose-stamped)
           (make-message
            "moveit_msgs/Constraints"
            :joint_constraints joint_constraints
            :position_constraints
            (map 'vector #'identity
                 (append
                  (list
                   (make-message
                    "moveit_msgs/PositionConstraint"
                    :weight 1.0
                    :link_name link-name
                    :header (make-message
                             "std_msgs/Header"
                             :frame_id (frame-id pose-stamped)
                             :stamp (stamp pose-stamped))
                    :constraint_region
                    (make-message
                     "moveit_msgs/BoundingVolume"
                     :primitives (vector
                                  (make-message
                                   "shape_msgs/SolidPrimitive"
                                   :type (roslisp-msg-protocol:symbol-code
                                          'shape_msgs-msg:solidprimitive :sphere)
                                   :dimensions (vector tolerance-radius)))
                     :primitive_poses
                     (vector (to-msg (cl-transforms-stamped:pose-stamped->pose pose-stamped))))))))
            :orientation_constraints
            (vector
             (make-message
              "moveit_msgs/OrientationConstraint"
              :weight 1.0
              :link_name link-name
              :header (make-message
                       "std_msgs/Header"
                       :frame_id (frame-id pose-stamped)
                       :stamp (stamp pose-stamped))
              :orientation
              (make-message
               "geometry_msgs/Quaternion"
               :x (cl-transforms:x (cl-transforms:orientation pose-stamped))
               :y (cl-transforms:y (cl-transforms:orientation pose-stamped))
               :z (cl-transforms:z (cl-transforms:orientation pose-stamped))
               :w (cl-transforms:w (cl-transforms:orientation pose-stamped)))
              :absolute_x_axis_tolerance tolerance-radius
              :absolute_y_axis_tolerance tolerance-radius
              :absolute_z_axis_tolerance tolerance-radius))))
         link-names poses-stamped)))
 
(defun make-trajectory-constraints (&key link-names reference-frames max-tilts
                                      reference-orientations)
  (make-message
   "moveit_msgs/TrajectoryConstraints"
   :constraints
   (vector
    (make-message
     "moveit_msgs/Constraints"
     :orientation_constraints
     (cond (max-tilts
            (map
             'vector
             (lambda (link-name reference-frame max-tilt reference-orientation)
               (make-message
                "moveit_msgs/OrientationConstraint"
                :header (make-message "std_msgs/Header"
                                      :stamp (roslisp:ros-time)
                                      :frame_id reference-frame)
                :orientation (make-message "geometry_msgs/Quaternion"
                                           :x (cl-transforms:x reference-orientation)
                                           :y (cl-transforms:y reference-orientation)
                                           :z (cl-transforms:z reference-orientation)
                                           :w (cl-transforms:w reference-orientation))
                :link_name link-name
                :absolute_x_axis_tolerance max-tilt
                :absolute_y_axis_tolerance max-tilt
                :absolute_z_axis_tolerance pi
                :weight 5.0))
             link-names reference-frames max-tilts reference-orientations))
           (t (vector)))))))

(defun check-base-pose-validity (pose-stamped)
  (with-lock-held (*moveit-pose-validity-check-lock*)
    (let* ((pose-stamped-oc (cl-transforms-stamped:transform-pose-stamped
                             *transformer*
                             :pose (copy-pose-stamped
                                    pose-stamped
                                    :stamp (roslisp:ros-time)) ; <- use current time
                             :target-frame *odom-frame*
                             :timeout *tf-default-timeout*))
           (origin (cl-transforms:origin pose-stamped-oc))
           (orientation (cl-transforms:orientation pose-stamped-oc)))
      (let ((adv (roslisp:advertise "/dhdhdh" "geometry_msgs/PoseStamped")))
        (roslisp:publish adv (to-msg pose-stamped-oc)))
      (let ((result
              (roslisp:call-service
               "/check_state_validity"
               'moveit_msgs-srv:getstatevalidity
               :group_name "both"
               :robot_state
               (make-message
                "moveit_msgs/RobotState"
                :multi_dof_joint_state
                (make-message
                 "moveit_msgs/MultiDOFJointState"
                 :header
                 (make-message
                  "std_msgs/Header"
                  :frame_id (concatenate 'string "/"
                                         (frame-id pose-stamped-oc)))
                 :joint_names (vector "virtual_joint")
                 :joint_transforms
                 (vector (make-message
                          "geometry_msgs/Transform"
                          :translation
                          (make-message
                           "geometry_msgs/Vector3"
                           :x (cl-transforms:x origin)
                           :y (cl-transforms:y origin)
                           :z (cl-transforms:z origin))
                          :rotation
                          (make-message
                           "geometry_msgs/Quaternion"
                           :x (cl-transforms:x orientation)
                           :y (cl-transforms:y orientation)
                           :z (cl-transforms:z orientation)
                           :w (cl-transforms:w orientation)))))))))
        (with-fields (valid) result
          valid)))))
