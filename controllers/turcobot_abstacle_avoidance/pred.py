import torch
import torchvision.transforms as transforms
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

from myModel import FloorSegmentationNet


IMAGE_RESIZE_TO = (256, 256)

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# model = torch.load(
#     "modelAndWeights_0421_allDataTogether_100epoch.pt",
#     map_location=device,
#     weights_only=False
# )
# model_name = "result_model_0421_allDataTogether_100epoch.pth"
# model_name = "result_model_0421_allDataTogether.pth"
# model_name = "result_model.pth"
model_name = "result_model(1).pth"
model = FloorSegmentationNet()
model.load_state_dict(torch.load(model_name, map_location=device))
model.eval()

# Common transform for images
img_transform = transforms.Compose([
    transforms.Resize(IMAGE_RESIZE_TO),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],  # From ImageNet
                         std=[0.229, 0.224, 0.225]),
])

def predict_image(model, image_path=None, image_array=None, show=False):
    if image_array is not None:
        image = Image.fromarray(image_array)
    else:
        image = Image.open(image_path).convert('RGB')

    input_tensor = img_transform(image).unsqueeze(0).to(device)

    with torch.no_grad():
        output = model(input_tensor)
        output = torch.sigmoid(output)
        pred_mask = output.squeeze().cpu().numpy()

        # Threshold the prediction at 0.5
        binary_result = (pred_mask > 0.5).astype(np.uint8) * 255  # 0 or 255


    # Show the original image and predicted mask
    if show == True:
        plt.figure(figsize=(10,4))
        plt.subplot(1, 2, 1)
        plt.imshow(image.resize(IMAGE_RESIZE_TO))
        plt.title("Input Image")

        plt.subplot(1, 2, 2)
        plt.imshow(binary_result, cmap='gray')
        plt.title("Predicted Floor Mask")
        plt.show()
    
    return binary_result
