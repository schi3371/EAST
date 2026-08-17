import subprocess
import os
import sys
import time
import shutil
from pathlib import Path

def remove_directory_with_retry(path, max_retries=5, delay=1):
    """Remove a directory with retry mechanism for Windows"""
    for i in range(max_retries):
        try:
            if os.path.exists(path):
                shutil.rmtree(path, ignore_errors=True)
            if not os.path.exists(path):
                return True
        except Exception as e:
            print(f"Attempt {i+1} failed to remove {path}: {str(e)}")
            if i < max_retries - 1:
                time.sleep(delay)
                continue
            else:
                print(f"Failed to remove {path} after {max_retries} attempts")
                return False
    return False

def clean_build_artifacts():
    """Clean up build artifacts with proper error handling"""
    directories_to_clean = ['build', 'dist']
    files_to_clean = ['runtime_hook.py', 'Ortho-Sim.spec', 'EAST.spec']
    
    # Clean directories
    for dir_path in directories_to_clean:
        if os.path.exists(dir_path):
            print(f"Cleaning directory: {dir_path}")
            remove_directory_with_retry(dir_path)
    
    # Clean files
    for file_path in files_to_clean:
        if os.path.exists(file_path):
            try:
                os.remove(file_path)
                print(f"Removed file: {file_path}")
            except Exception as e:
                print(f"Failed to remove {file_path}: {str(e)}")

def copy_images_folder(src_dir, dst_dir):
    """Copy images folder to destination directory"""
    try:
        images_src = os.path.join(src_dir, 'images')
        images_dst = os.path.join(dst_dir, 'images')
        
        # Create images directory in dist if it doesn't exist
        if not os.path.exists(images_dst):
            os.makedirs(images_dst)
        
        # Copy all files from images folder
        for item in os.listdir(images_src):
            src_item = os.path.join(images_src, item)
            dst_item = os.path.join(images_dst, item)
            if os.path.isfile(src_item):
                shutil.copy2(src_item, dst_item)
                print(f"Copied {item} to dist/images/")
            elif os.path.isdir(src_item):
                shutil.copytree(src_item, dst_item, dirs_exist_ok=True)
                print(f"Copied directory {item} to dist/images/")
        
        print("Successfully copied images folder to dist directory")
        return True
    except Exception as e:
        print(f"Error copying images folder: {str(e)}")
        return False

# Get current directory and convert to proper path format
cur_dir = Path(os.getcwd()).resolve()
print(f"Current directory: {cur_dir}")

# Get Python environment site-packages
python_path = Path(sys.executable).parent / "Lib" / "site-packages"
site_packages = python_path.as_posix()
print(f"Site packages directory: {site_packages}")

# Create runtime hook file
runtime_hook_content = """
def _setup_numpy():
    import os
    import sys
    import importlib.util

    # If numpy is already imported, return
    if 'numpy' in sys.modules:
        return

    # Find numpy in the frozen package
    numpy_path = None
    for path in sys.path:
        possible_path = os.path.join(path, 'numpy')
        if os.path.exists(possible_path):
            numpy_path = possible_path
            break

    if numpy_path:
        # Add numpy path to sys.path if not already there
        if numpy_path not in sys.path:
            sys.path.insert(0, numpy_path)

# Call the setup function
_setup_numpy()
"""

# Clean up before starting
print("\nCleaning up old build artifacts...")
clean_build_artifacts()

try:
    print("\nCreating runtime hook...")
    with open("runtime_hook.py", "w") as f:
        f.write(runtime_hook_content)

    pyinstaller_command = [
        "pyinstaller",
        "--clean",  # Clean PyInstaller cache
        "Ortho-Sim.py",  # Main script path
        "--name", "EAST",
        "--onefile",
        "--windowed",  # Hide console window
        f"--icon={cur_dir}/images/icon.ico",
        "--paths", site_packages,  # Add site-packages to path
        "--noconfirm",  # Replace existing spec without asking
        
        # Add all files from images directory
        f"--add-data", f"{cur_dir}/images/*;images/",
        f"--add-data", f"{cur_dir}/tester_config.json;.",
        
        # Required hidden imports for PyQtGraph
        "--hidden-import", "pyqtgraph.graphicsItems.ViewBox.axisCtrlTemplate_pyqt5",
        "--hidden-import", "pyqtgraph.graphicsItems.PlotItem.plotConfigTemplate_pyqt5",
        "--hidden-import", "pyqtgraph.imageview.ImageViewTemplate_pyqt5",
        
        # Required imports for ODrive
        "--hidden-import", "fibre",
        "--hidden-import", "fibre.utils",
        "--hidden-import", "odrive",
        "--hidden-import", "odrive.utils",
        "--hidden-import", "odrive.enums",
        
        # Required imports for Phidget
        "--hidden-import", "Phidget22",
        "--hidden-import", "Phidget22.PhidgetException",
        "--hidden-import", "Phidget22.Devices.VoltageRatioInput",
        
        # Additional PyQt5 imports that might be needed
        "--hidden-import", "PyQt5.QtCore",
        "--hidden-import", "PyQt5.QtGui",
        "--hidden-import", "PyQt5.QtWidgets",
        
        # NumPy related imports and configurations
        "--hidden-import", "numpy",
        "--hidden-import", "numpy.core",
        "--hidden-import", "numpy.core._methods",
        "--hidden-import", "numpy.lib",
        "--hidden-import", "numpy.lib.format",
        "--hidden-import", "numpy.random",
        "--hidden-import", "numpy.linalg",
        "--hidden-import", "numpy.linalg._umath_linalg",
        
        # Additional hooks directory
        "--additional-hooks-dir", ".",
        
        # Collect all required packages
        "--collect-all", "pyqtgraph",
        "--collect-all", "odrive",
        "--collect-all", "Phidget22",
        
        # Additional options for better compatibility
        "--disable-windowed-traceback",  # Disable traceback for cleaner error handling
        
        # Runtime hooks
        "--runtime-hook", "runtime_hook.py",
        
        # Additional PyInstaller options for better NumPy handling
        "--copy-metadata", "numpy",
        "--copy-metadata", "pyqtgraph",
        "--copy-metadata", "odrive",
        "--copy-metadata", "Phidget22"
    ]

    print("\nRunning PyInstaller with command:")
    print(" ".join(pyinstaller_command))

    result = subprocess.run(pyinstaller_command, check=True, capture_output=True, text=True)
    print("\nBuild output:")
    print(result.stdout)
    
    # Copy images folder to dist directory
    print("\nCopying images folder to dist directory...")
    if copy_images_folder(cur_dir, os.path.join(cur_dir, 'dist')):
        print("Images folder copied successfully!")
    else:
        print("Failed to copy images folder")
    
    print("\nBuild completed successfully!")

except subprocess.CalledProcessError as e:
    print("\nBuild failed with error:")
    print(e.stdout)
    print("\nError output:")
    print(e.stderr)
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
finally:
    print("\nCleaning up temporary files...")
    if os.path.exists("runtime_hook.py"):
        try:
            os.remove("runtime_hook.py")
        except Exception as e:
            print(f"Failed to remove runtime_hook.py: {str(e)}")
    print("Cleanup completed.")
