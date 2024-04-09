from PIL import Image


def onImage(image: Image):
    if image is None:
        return

    image.show()
