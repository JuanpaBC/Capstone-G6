import cv2
import torch
from torchvision import transforms
from ultralytics import YOLO
model_file_name = "best.pt"
# Load the pre-trained model
model = YOLO()
model = model.load(model_file_name)

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
    
    for a in predictions:
        c = a.boxes
        print(c)
        area = cv2.contourArea(c)
        if area > 300:
            M = cv2.moments(c)
            if M["m00"] == 0:
                M["m00"] = 1
            x = int(M["m10"] / M["m00"])
            y = int(M["m01"] / M["m00"])
            cv2.circle(frame, (x, y), 7, (255, 0, 255), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, '{},{}'.format(
                x, y), (x+10, y), font, 0.75, (255, 0, 255), 1, cv2.LINE_AA)
            nuevoContorno = cv2.convexHull(c)
            cv2.circle(frame, (x, y), max(
                nuevoContorno[:, 0, 0].tolist()) - x, (0, 0, 255), 2)
            cv2.drawContours(
                frame, [nuevoContorno], 0, (0, 255, 0), 3)

    # Display the resulting frame
    cv2.imshow('Object Detection', frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
