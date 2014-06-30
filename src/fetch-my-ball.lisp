(in-package :fetch-my-ball)

(defun init-driver ()
  (roslisp-utilities:startup-ros)
  (setf *pub* (advertise "joint_command" "nxt_msgs/JointCommand")))

(defun drive-forward ()
  "Drive forward"
  (send-joint-commands '("r_motor_joint" "l_motor_joint") '(0.9 0.9)))

(defun stop-driving ()
  "Stop driving"
  (send-joint-commands '("r_motor_joint" "l_motor_joint") '(0.0 0.0)))

(defun send-joint-command (joint effort)
  (send-joint-commands (list joint) (list effort)))

(defun send-joint-commands (joints efforts)
  (mapcar (lambda (joint effort)
    (publish-msg *pub* :name joint :effort effort)) joints efforts))

(defun close-gripper ()
  (send-joint-command "gripper" -0.6))

(defun open-gripper ()
  (send-joint-command "gripper" 0.6))

(defun relax-gripper ()
  (send-joint-command "gripper" 0.0))
