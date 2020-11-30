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

# install fonts
mkdir -p $HOME/local_fonts/fonts
pushd $HOME/local_fonts/
git clone git@github.com:python-pillow/Pillow.git
cp Pillow/Tests/fonts/* fonts/
rm -rf Pillow
popd
echo '' >> ~/.bashrc
echo '# pillow fonts' >> ~/.bashrc
echo 'export XDG_DATA_DIRS="$HOME/local_fonts/"' >> ~/.bashrc
