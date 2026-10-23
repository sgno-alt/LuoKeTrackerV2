@echo off
cd /d "%~dp0"
call "%~dp0_env.bat" root
if errorlevel 1 goto :end
echo 使用 Anaconda：%CONDA_ROOT%
echo.

call "%CONDA_ROOT%\condabin\conda.bat" env list | findstr /i /r /c:"^%ENV_NAME% " >nul
if errorlevel 1 (
  echo [1/3] 创建环境 %ENV_NAME% ^(Python 3.11^)...
  call "%CONDA_ROOT%\condabin\conda.bat" create -n %ENV_NAME% python=3.11 -y
  if errorlevel 1 ( echo [错误] 创建环境失败 & goto :end )
) else (
  echo [1/3] 环境 %ENV_NAME% 已存在，跳过创建
)

echo.
echo [2/3] 安装依赖（清华镜像）...
call "%CONDA_ROOT%\Scripts\activate.bat" %ENV_NAME%
python -m pip install -U opencv-python numpy mss pillow -i https://pypi.tuna.tsinghua.edu.cn/simple
if errorlevel 1 ( echo [错误] 依赖安装失败 & goto :end )

echo.
echo [3/3] 自检...
python -c "import cv2, mss, PIL, numpy; ok = 'Win32 UI' in cv2.getBuildInformation(); print('OpenCV', cv2.__version__, '窗口功能:', '正常' if ok else '缺失(装成了headless版)')"
echo.
echo 完成！以后双击「启动.bat」即可。
:end
pause
