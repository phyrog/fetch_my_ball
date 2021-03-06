(in-package :1d-filter)

(defparameter *backlog* nil)
(defparameter *thread-lock* (make-lock))
(defparameter *thread-lock-condition* (make-condition-variable))
(defparameter *listener-node* nil)
(defparameter *fn* nil)
(defparameter *radius* nil)
(defparameter *weight* nil)
(defparameter *producer-thread* nil)
(defparameter *consumer-thread* nil)
(defparameter *subscriber* nil)

(defun main (fn radius weight in-topic out-topic in-topic-type out-topic-type)
  "Runs the specified filter function with the given radius and weight on the
   values of the in-topic and publishes the values to the out-topic"
  (setf *in-topic* in-topic)
  (setf *in-topic-type* in-topic-type)
  (setf *out-topic* out-topic)
  (setf *out-topic-type* out-topic-type)
  (setf *fn* fn)
  (setf *radius* radius)
  (setf *weight* weight)
  (unless (eq roslisp::*node-status* :running) (roslisp-utilities:startup-ros))
  (setf *producer-thread* (make-thread #'produce-values :name "foo"))
  (setf *consumer-thread* (make-thread #'consume-values)))

(defun produce-values ()
  "Runs the producer, that consumes values from the topic and puts them into the consumer queue"
  (setf *subscriber* (subscribe *in-topic* *in-topic-type* (lambda (value)
    (with-lock-held (*thread-lock*)
      (let ((v (if (< (with-fields ((range range)) value range) 0.5) (append *backlog* `(,value)) *backlog*)))
        (setf *backlog* v))
      (condition-notify *thread-lock-condition*))))))

(defun consume-values ()
  "Consumes the values from the consumer queue and produces 
   values on the out-topic by applying the given filter function"
  (let ((pub (advertise *out-topic* *out-topic-type*)))
    (acquire-lock *thread-lock* t)
    (loop
      (let ((value-list nil))
        (if (>= (length *backlog*) *radius*)
          (progn
            (setf value-list (subseq *backlog* 0 *radius*))
            (setf *backlog* (cdr *backlog*))))
        (if value-list
          (publish pub (apply *fn* `(,value-list)))))
      (condition-wait *thread-lock-condition* *thread-lock*))))

(defun end-filter ()
  "Stops the filter"
  (unsubscribe *subscriber*)
  (destroy-thread *producer-thread*)
  (destroy-thread *consumer-thread*))

(defun avg (l)
  "Filter function for Float64 topics that averages the values in the given radius"
  (make-msg "std_msgs/Float64" :data (/ (apply #'+ (mapcar (lambda (a) (with-fields ((data data)) a data)) l)) *radius*)))

(defun average-dist (l)
  "Filter function for sensor_msgs/Range topics that applies weights to all values in the given radius"
  (let ((vals (mapcar (lambda (a w)
                        (with-fields ((range range)) a (* range w))) l *weight*)))
    (make-msg "std_msgs/Float64" :data (apply #'+ vals))))

(defun filter-smooth ()
  "Run the average-dist filter on the topic /ultrasonic_sensor publishing on /avg_range with a radius of 7"
  (main #'average-dist 7 '(0.080808 0.161616 0.333333 0 0.333333 0.161616 0.080808) "/ultrasonic_sensor" "/avg_range" "sensor_msgs/Range" "std_msgs/Float64"))
