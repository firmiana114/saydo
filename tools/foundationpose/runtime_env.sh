#!/usr/bin/env bash
# Source after activating the independent environment, or use its Python
# directly.
SAYDO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export CONDA_PREFIX="${FOUNDATIONPOSE_ENV:-$SAYDO_ROOT/work/foundationpose_env}"
export CUDA_HOME="$CONDA_PREFIX"
export PATH="$CONDA_PREFIX/bin:$PATH"
export CC="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-cc"
export CXX="$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-c++"
export CUDAHOSTCXX="$CXX"
# 下面几个路径变量很长，每个路径占一行。双引号内行尾的反斜杠表示
# “接下一行”，续行必须顶格写，否则行首的空格会进入变量值。
export CPLUS_INCLUDE_PATH="$CONDA_PREFIX/include/eigen3:\
$CONDA_PREFIX/targets/x86_64-linux/include\
${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
export TORCH_CUDA_ARCH_LIST="8.9"
export MAX_JOBS=4
export PYOPENGL_PLATFORM=egl
export MPLBACKEND=Agg
export TORCH_FORCE_WEIGHTS_ONLY_LOAD=1
export TORCH_EXTENSIONS_DIR="$SAYDO_ROOT/work/foundationpose_setup/\
torch_extensions"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:\
$CONDA_PREFIX/targets/x86_64-linux/lib\
${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$SAYDO_ROOT/work/foundationpose_runtime:\
$SAYDO_ROOT/work/foundationpose_runtime/bundlesdf:\
$SAYDO_ROOT/work/foundationpose_runtime/bundlesdf/mycuda:\
$SAYDO_ROOT/work/foundationpose_runtime/mycpp/build\
${PYTHONPATH:+:$PYTHONPATH}"
