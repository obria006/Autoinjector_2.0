from setuptools import find_packages, setup
setup(
    name='Autoinjector-2',
    version="1.0.0a1",
    description="Python package for automated, robotic microinjection system developed for Human Technopole.",
    long_description=("Autoinjector-2 is a vision guided robotic microinjection "
    "system developed for the Taverna Group at Human Technopole. The goal of "
    "Autoinjector-2 is to enhance the automation of the initial Autoinjector-1 "
    "system to enable faster and higher-throughput microinjection experiments "
    "into embryonic mouse tissue and human brain organoids. Autoinjector-2 "
    "strives to achieve this goal by implimenting a number of improvements: "
    "1. 3D microscope-manipulator calibration to enable 3D microinjection "
    "targetting. 2. Computer vision assisted calibration to increase the speed "
    "of the periodically repeated calibration process. 3. Computer vision "
    "assisted tissue detection to improve the speed and consistency of the "
    "frequently repeated target annotation process. 4. Computer vision "
    "annotation tracking to faciliate accurate targetting of injection "
    "while the tissue is displacing during injections. 5. High-pressure "
    "unclogging to reduce the frequency of needing to change microinjection "
    "needles"),
    url="https://github.com/obria006/Autoinjector_2.0.git",
    author="Jacob O'Brien",
    author_email="obrienja7@gmail.com",
    license="MIT",
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3',
        "Operating System :: Microsoft :: Windows :: Windows 10",
    ],
    python_requires=">=3.7",
    install_requires=[
        "matplotlib",
        "numpy",
        "onnxruntime",
        "opencv-python",
        "pandas",
        "Pillow",
        "pymmcore",
        "PyQt6",
        "pyserial",
        "pywin32",
        "PyYAML",
        "scikit-image",
        "scipy",
        "sensapex",
        "tifffile",
        "torch",
        "torchvision"
    ],
    packages=find_packages()
)