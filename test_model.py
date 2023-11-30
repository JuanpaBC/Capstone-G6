import cv2
import torch
from torchvision import transforms
from ultralytics import YOLO
model_file_name = "best.pt"
# Load the pre-trained model
model = YOLO(model_file_name)
# Set up the camera
cap = cv2.VideoCapture(0)  # 0 corresponds to the default camera

# Define the transformation for the input image
transform = transforms.Compose([
    transforms.ToTensor(),
])

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Make predictions
    with torch.no_grad():
        predictions = model(frame)
    
    for result in predictions:
        result_bboxes = result.boxes.xyxy
        print(result)
        for result_bbox in result_bboxes:
            box = result_bbox[:4].int().tolist()
            frame = cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
            label = "castana"
            frame = cv2.putText(frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    # Display the resulting frame with predictions
    cv2.imshow('Object Detection', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
