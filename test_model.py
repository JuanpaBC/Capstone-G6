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
        result_bboxes = result.boxes
        print(result)
        for result_bbox in result_bboxes:
            b_coordinates = result_bbox.xyxy[0]
            too_much_overlap = False
            b_center = result_bbox.xywh[0]  # get box coordinates in (top, left, bottom, right) format
            box = b_coordinates[:4].int().tolist()
            for other in result_bboxes:
                o_coordinates = other.xyxy[0]
                if (b_coordinates.cpu().numpy() == o_coordinates.cpu().numpy()).all():
                  continue
                # Calculate the intersection coordinates
                x1 = max(b_coordinates[0].cpu().numpy(), o_coordinates[0].cpu().numpy())
                y1 = max(b_coordinates[1].cpu().numpy(), o_coordinates[1].cpu().numpy())
                x2 = min(b_coordinates[2].cpu().numpy(), o_coordinates[2].cpu().numpy())
                y2 = min(b_coordinates[3].cpu().numpy(), o_coordinates[3].cpu().numpy())

                # Calculate area of intersection
                intersection_area = max(0, x2 - x1) * max(0, y2 - y1)
                b_area = b_center[2].cpu().numpy() * b_center[3].cpu().numpy()
                if intersection_area / b_area > 0.5:
                    too_much_overlap = True
                    break
            if not too_much_overlap:
                confidence = result_bbox.conf
                if confidence[0].cpu().numpy() > 0.5:
                    frame = cv2.circle(frame, (int(b_center[0].cpu().numpy()), int(b_center[1].cpu().numpy())), 5, (0, 0, 255), -1)
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
