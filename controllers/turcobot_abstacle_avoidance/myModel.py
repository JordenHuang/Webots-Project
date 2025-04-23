import torch.nn as nn
import torchvision.models as models

class FloorSegmentationNet(nn.Module):
    def __init__(self, n_classes=1):
        super().__init__()
        resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        self.encoder = nn.Sequential(*list(resnet.children())[:-2])  # Keep until layer4 (out: 512x4x2 for 128x64 input)

        # Decoder: upsample back to 128x256
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(512, 256, kernel_size=2, stride=2),  # 4x8 → 8x16
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(256, 128, kernel_size=2, stride=2),  # 8x16 → 16x32
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(128, 64, kernel_size=2, stride=2),   # 16x32 → 32x64
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(64, 32, kernel_size=2, stride=2),    # 32x64 → 64x128
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(32, 16, kernel_size=2, stride=2),    # 64x128 → 128x256
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 1, kernel_size=1)                         # final 1-channel prediction
        )


        self.final_activation = nn.Sigmoid()

    def forward(self, x):
        x = self.encoder(x)
        x = self.decoder(x)
        return x
        # return self.final_activation(x)
