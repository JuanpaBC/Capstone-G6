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

    # Transform the frame to tensor
    input_tensor = transform(frame).unsqueeze(0)

    # Make predictions
    with torch.no_grad():
        predictions = model(input_tensor)
    
    for result in predictions:
        result_bbox = result.boxes.xyxy
        result_names = result.names
        result_prob = result.probs
        for result, name in zip(result_bbox, result_names):
            print(result)
            box = result[:4].int().tolist()
            frame = cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
            label = f"{name}"
            frame = cv2.putText(frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    # Display the resulting frame with predictions
    cv2.imshow('Object Detection', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
