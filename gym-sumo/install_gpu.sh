#!/bin/bash
# install torch, torch-geometric
conda install pytorch==1.6.0 torchvision==0.7.0 cudatoolkit=10.2 -c pytorch
pip install torch-scatter -f https://pytorch-geometric.com/whl/torch-1.6.0+cu102.html
pip install torch-sparse -f https://pytorch-geometric.com/whl/torch-1.6.0+cu102.html
pip install torch-cluster -f https://pytorch-geometric.com/whl/torch-1.6.0+cu102.html
pip install torch-spline-conv -f https://pytorch-geometric.com/whl/torch-1.6.0+cu102.html
pip install torch-geometric

# other requirement packages
# other requirement packages
rm -r ./*.egg-info/
pip install -e .

# install fonts
mkdir -p "$HOME"/local_fonts/fonts
pushd "$HOME"/local_fonts/ || exit
git clone git@github.com:python-pillow/Pillow.git
cp Pillow/Tests/fonts/* fonts/
rm -rf Pillow
popd || exit
{
        echo ''
        echo '# pillow fonts'
        # shellcheck disable=SC2016
        echo 'export XDG_DATA_DIRS="$HOME/local_fonts/:$XDG_DATA_DIRS"'
} >> ~/.bashrc
