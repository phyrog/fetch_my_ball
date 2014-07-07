(defsystem fetch-my-ball
  :author "Tom Gehrke <tom.gehrke@uni-bremen.de>"
  
  :depends-on (roslisp
               roslisp-utilities
               nxt_msgs-msg
               sensor_msgs-msg
               alexandria)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "fetch-my-ball" :depends-on ("package"))))))
