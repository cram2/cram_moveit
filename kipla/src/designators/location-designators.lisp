;;;
;;; Copyright (c) 2010, Lorenz Moesenlechner <moesenle@in.tum.de>
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
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
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
;;;

(in-package :kipla-reasoning)

;;; Location designators are resolved a little bit differently than
;;; object designators (at least for now.) To resolve, the
;;; cram/reasoning prolog predicate desig-loc is used. All solutions
;;; are provided an can be accessed with next-solution. A mechanism is
;;; provided to post-process the solutions from reasoning, e.g. to
;;; sort according to eucledian distance.

(defconstant +costmap-n-samples+ 3)

(defgeneric make-location-proxy (type value)
  (:documentation "Creates a location proxy of `type' and initializes
  it with `value'."))

(defgeneric location-proxy-current-solution (proxy)
  (:documentation "Returns the current solution of the proxy"))

(defgeneric location-proxy-next-solution (proxy)
  (:documentation "Returns the next solution of the proxy object or
  NIL if no more solutions exist."))

(defgeneric location-proxy-precedence-value (proxy)
  (:documentation "Returns a number that indicates the proxie's
  precedence. Lower numbers correspond to lower precedence."))

(defgeneric location-proxy-solution->pose (desig solution)
  (:method (desig (solution cl-transforms:pose))
    solution))

(defclass pose-location-proxy ()
  ((pose :accessor location-proxy-current-solution :initarg :pose))
  (:documentation "Proxy class for designators that contain poses."))

(defclass point-location-proxy ()
  ((point :accessor location-proxy-current-solution :initarg :pose))
  (:documentation "Proxy class for designators that contain poses."))

(defclass costmap-location-proxy (point-location-proxy)
  ((next-solutions :accessor :next-solutions :initform nil
                   :documentation "List of the next solution. We want
                   to minimize driving distances, so we always
                   generate a bunch of solutions, order them by
                   distance to the robot and always chose the closest
                   one when generating a new solution.")
   (costmap :initarg :costmap :reader costmap))
  (:documentation "Proxy class to generate designator solutions from a
  costmap."))

(defclass location-designator (designator designator-id-mixin)
  ((current-solution :reader current-solution :initform nil)))

(register-designator-type location location-designator)

(defmethod reference ((desig location-designator))
  (with-slots (data current-solution) desig
    (unless current-solution
      (setf data (mapcar (curry #'apply #'make-location-proxy)
                         (sort (mapcar (curry #'var-value '?value)
                                       (force-ll (prolog `(desig-loc ,desig ?value))))
                               #'> :key #'location-proxy-precedence-value)))
      (assert data () (format nil "Unable to resolve designator `~a'" desig))
      (setf current-solution (location-proxy-solution->pose
                              desig
                              (location-proxy-current-solution (car data))))
      (assert current-solution () (format nil "Unable to resolve designator `~a'" desig)))
    current-solution))

(defmethod next-solution ((desig location-designator))
  ;; Make sure that we initialized the designator properly
  (unless (slot-value desig 'current-solution)
    (reference desig))
  (with-slots (data) desig
    (or (successor desig)
        (let ((new-desig (make-designator 'location (description desig))))
          (or
           (let ((next (location-proxy-next-solution (car data))))
             (when next
               (setf (slot-value new-desig 'data) data)
               (setf (slot-value new-desig 'current-solution)
                     (location-proxy-solution->pose new-desig next))
               (equate desig new-desig)))
           (when (cdr data)
             (let ((next (location-proxy-current-solution (cadr data))))
               (when next
                 (setf (slot-value new-desig 'data) (cdr data))
                 (setf (slot-value new-desig 'current-solution)
                       (location-proxy-solution->pose new-desig next))
                 (equate desig new-desig)))))))))

;; Todo: make the poses stamped
(defmethod make-location-proxy ((type (eql 'point)) (value cl-transforms:3d-vector))
  (make-instance 'pose-location-proxy
                 :pose (cl-transforms:make-pose
                        value (cl-transforms:make-quaternion 0 0 0 1))))

(defmethod make-location-proxy ((type (eql 'pose)) (value cl-transforms:pose))
  (make-instance 'pose-location-proxy
                 :pose value))

(defmethod location-proxy-next-solution ((proxy pose-location-proxy))
  nil)

(defmethod location-proxy-precedence-value ((proxy pose-location-proxy))
  1)

(defmethod make-location-proxy ((type (eql 'costmap)) (val location-costmap))
  (make-instance 'costmap-location-proxy :costmap val))

(defmethod initialize-instance :after ((proxy costmap-location-proxy) &key &allow-other-keys)
  (location-proxy-next-solution proxy))

(defmethod location-proxy-next-solution ((proxy costmap-location-proxy))
  (with-slots (point next-solutions costmap)
      proxy
    (let ((solutions (or next-solutions
                         (loop repeat +costmap-n-samples+
                               collecting (gen-costmap-sample costmap)))))
      ;; Todo: take the closest next solution, not the first one
      ;; Todo: add orientation
      (setf next-solutions (cdr solutions))
      (setf point (car solutions)))))

(defmethod location-proxy-precedence-value ((proxy costmap-location-proxy))
  0)

(defmethod location-proxy-solution->pose (desig (solution cl-transforms:3d-vector))
  (with-vars-bound (?o)
      (lazy-car (prolog `(desig-orientation ,desig ,solution ?o)))
    (cl-transforms:make-pose
     solution
     (or (unless (is-var ?o)
           ?o)
         (cl-transforms:make-quaternion 0 0 0 1)))))