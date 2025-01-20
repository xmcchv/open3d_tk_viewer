# open3d tkiner pointcloud viewer


## environment setup 

```python
pip install numpy open3d 
```

## quick start
1. pointcloud viewer
setup variable "directory_path" in viewer.py and run ```python viewer.py``` in terminal

2. calculate eular angles to xoz plane
setup variable "file_path" in fitplane.py and run ```python fitplane.py``` in terminal, more variables are available in fitplane.py

## modify
if you have different file format, you should implement your own ```load_point_cloud``` function