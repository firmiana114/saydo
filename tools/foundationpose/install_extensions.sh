#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/runtime_env.sh"
cd "$SAYDO_ROOT"
python -c \
  'import torch; assert torch.cuda.is_available(); print(torch.__version__)'
python -m pip install 'setuptools<81' wheel ninja
# 双引号内行尾的反斜杠表示“接下一行”，续行必须顶格写，
# 否则行首的空格会进入变量值。
KAOLIN_WHEEL="$SAYDO_ROOT/work/foundationpose_setup/\
kaolin-0.18.0-cp311-cp311-linux_x86_64.whl"
if [ -f "$KAOLIN_WHEEL" ]; then
  python -m pip install "$KAOLIN_WHEEL" \
    -c tools/foundationpose/constraints_msi.txt
else
  python -m pip install kaolin==0.18.0 \
    -f https://nvidia-kaolin.s3.us-east-2.amazonaws.com/torch-2.5.1_cu124.html \
    -c tools/foundationpose/constraints_msi.txt
fi
python -m pip install --no-build-isolation \
  work/foundationpose_deps/sam-source \
  work/foundationpose_deps/nvdiffrast-source
python -m pip install --no-build-isolation \
  work/foundationpose_deps/pytorch3d-source
cmake -S work/foundationpose_runtime/mycpp \
  -B work/foundationpose_runtime/mycpp/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DCMAKE_CXX_COMPILER="$CXX" \
  -DPYBIND11_PYTHON_EXECUTABLE="$CONDA_PREFIX/bin/python"
cmake --build work/foundationpose_runtime/mycpp/build -j4
cd work/foundationpose_runtime/bundlesdf/mycuda
python setup.py build_ext --inplace
cd "$SAYDO_ROOT"
python tools/foundationpose/check_gpu.py
