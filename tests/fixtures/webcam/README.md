# Webcam regression fixture

`open_hand_loss.mp4.b64` decodes to a 320x240, 30 fps video containing 30
open-hand frames followed by six blank frames. Tests decode it into a temporary
directory so Git stores a reviewable text representation.

The hand image is cropped and downsampled from MediaPipe's `right_hands.jpg`
test asset, distributed by the MediaPipe Authors under Apache-2.0. The blank
tail and video encoding were generated for ACT Lab. It contains no face, room,
screen, audio, or ACT Lab operator recording. Encoded-video SHA-256:
`59fc6063feaef2d444ec01d88f702c9bd2c1b18b960781bb89dc56ae863a0b63`.

Source: `https://storage.googleapis.com/mediapipe-assets/right_hands.jpg`
