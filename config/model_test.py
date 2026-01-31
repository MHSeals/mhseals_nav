from ultralytics import YOLO
import cv2

model = YOLO('best.onnx')
cap = cv2.VideoCapture(0)
print(model.names)

while cap.isOpened():
    success, frame = cap.read()

    if success:
        results = model(frame, verbose=False)
        annotated_frame = results[0].plot()

        cv2.imshow('YOLO Object Detection', annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()