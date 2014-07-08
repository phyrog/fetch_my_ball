(defsystem fetch-my-ball
  :author "Tom Gehrke <tom.gehrke@uni-bremen.de>"
  
  :depends-on (;;1d-filter
               roslisp
               roslisp-utilities
               nxt_msgs-msg
               sensor_msgs-msg
               bordeaux-threads
               alexandria)
  :components
  ((:module "src"
    :components
    ((:file "package")
     (:file "fetch-my-ball" :depends-on ("package"))))))
