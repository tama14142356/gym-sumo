#!/bin/bash
# install torch, torch-geometric
pip install torch==1.6.0+cpu torchvision==0.7.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
pip install torch-scatter==latest+cpu -f https://pytorch-geometric.com/whl/torch-1.6.0.html
pip install torch-sparse==latest+cpu -f https://pytorch-geometric.com/whl/torch-1.6.0.html
pip install torch-cluster==latest+cpu -f https://pytorch-geometric.com/whl/torch-1.6.0.html
pip install torch-spline-conv==latest+cpu -f https://pytorch-geometric.com/whl/torch-1.6.0.html
pip install torch-geometric

# other requirement packages
rm -r *.egg-info/
pip install -e .
