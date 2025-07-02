import torch.nn as nn
import torchvision.models as models

class FloorSegmentationNet(nn.Module):
    def __init__(self, n_classes=1):
        super().__init__()
        resnet = models.resnet18(pretrained=True)
        self.encoder = nn.Sequential(*list(resnet.children())[:-2])  # Output: [B, 512, 8, 8] for 256×256 input

        # Decoder to upsample back to 256x256
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(512, 256, kernel_size=2, stride=2),  # 8x8 → 16x16
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(256, 128, kernel_size=2, stride=2),  # 16x16 → 32x32
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(128, 64, kernel_size=2, stride=2),   # 32x32 → 64x64
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(64, 32, kernel_size=2, stride=2),    # 64x64 → 128x128
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(32, 16, kernel_size=2, stride=2),    # 128x128 → 256x256
            nn.ReLU(inplace=True),
          nn.Conv2d(16, n_classes, kernel_size=1)                 # 1×1 conv to output 1-channel
        )

    def forward(self, x):
        x = self.encoder(x)  # [B, 512, 8, 8]
        x = self.decoder(x)  # [B, 1, 256, 256]
        return x

        # self.final_activation = nn.Sigmoid()  # if using BCELoss

# ==============================
import torch
import torchvision.transforms as transforms
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

IMAGE_RESIZE_TO = (256, 256)
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Common transform for images
img_transform = transforms.Compose([
    transforms.Resize(IMAGE_RESIZE_TO),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],  # From ImageNet
                         std=[0.229, 0.224, 0.225]),
])

# model_name = "../floor_seg_model/result_model_0507.pth"
model_name = "/home/jordenhuang/programs/models/bestModel_0702.pth"
model = FloorSegmentationNet()
model.load_state_dict(torch.load(model_name, map_location=device))
model.eval()

def predict_image(image_path=None, image_array=None, show=False):
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