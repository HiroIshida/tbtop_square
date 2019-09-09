
(defun bt-nize! (obj)
  (send obj :make-btmodel)
  (geo::_bt-set-margin (send obj :get :btmodel) -0.04))

(defun make-vase-points (hw-lst)
  (let ((lst-ret nil))
    (dolist (hw hw-lst)
      (let ((h (car hw))
            (w (cdr hw)))
        (push (float-vector w w h) lst-ret)
        (push (float-vector (- w) w h) lst-ret)
        (push (float-vector w (- w) h) lst-ret)
        (push (float-vector (- w) (- w) h) lst-ret)))
    lst-ret))

(setq *vase-pts* (make-vase-points `((0 . 30) (40 . 38) (60 . 41) (80 . 41.5))))
(setq *vase-tmp* (convex-hull-3d *vase-pts*))
(setq *box* (make-cube 80 80 10))
(send *box* :translate #f(0 0 75))
(setq *vase* (body- *vase-tmp* *box*))
(objects *vase*)

(defun gen-pc (N)
  (let ((cube (make-cube 1 1 1)) (lst-cube) (lst-vec))
    (dotimes (i N)
      (let ((vec (random-vector)))
        (setf [vec 0] (* 100 [vec 0])) 
        (setf [vec 1] (* 100 [vec 1]))
        (setf [vec 2] (+ (* 100 [vec 2]) 50))
          (let ((cube-tmp (copy-object cube)))
            (send cube-tmp :translate vec)
            (when (< (car (pqp-collision-distance *vase* cube-tmp)) 3)
                (push cube-tmp lst-cube)
                (push vec lst-vec)))))
    (cons lst-cube lst-vec)))

(let ((tmp (gen-pc 1000)))
  (setq *cubes* (car tmp))
  (setq *vecs* (cdr tmp)))

(setq *txt* "")
(dolist (vec *vecs*)
  (let ((line (concatenate string (string [vec 0]) ", " (string [vec 1]) ", " (string [vec 2]))))
    (setq *txt* (concatenate string *txt* line '(#\Newline)))))


(with-open-file (str "../model/vase.csv"
                       :direction :output)
  (format str *txt*))







