from setuptools import setup, find_packages, Extension
from Cython.Build import cythonize
import glob
import os

# Get all .py files inside 'nayanam' (excluding __init__.py)
py_files = [f for f in glob.glob("accident_detection/*.py") if not f.endswith("__init__.py")]

# Define Cython extensions dynamically
ext_modules = cythonize([
    Extension(f"accident_detection.{os.path.splitext(os.path.basename(file))[0]}", [file]) for file in py_files
])

# Define package data without hardcoding file names
# package_data = {
#     'accident_detection': [
#         'accident_detection/best.pt',
#         'accident_detection/best.engine',
#         'nayanam/best.onnx',
#         'nayanam/best_openvino_model/**',  # Wildcard handled via MANIFEST.in
#     ]
# }

setup(
    name='accident_detection',
    version='1.0.0',
    packages=find_packages(),
    ext_modules=ext_modules,  # Compile Cython modules
    include_package_data=True,  # Important to include non-Python files
    python_requires='>=3.8',
)
