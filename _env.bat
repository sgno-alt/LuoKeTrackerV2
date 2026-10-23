@echo off
rem ============================================================
rem  公共脚本：查找 Anaconda 并激活 luoke 环境（被其他 bat 调用）
rem  用法：call _env.bat        查找并激活环境
rem        call _env.bat root   只查找 Anaconda 安装位置
rem ============================================================
set "ENV_NAME=luoke"
set "CONDA_ROOT="

rem 1) 手动指定：在本文件夹放 conda_root.txt，里面写 Anaconda 安装路径
if exist "%~dp0conda_root.txt" set /p CONDA_ROOT=<"%~dp0conda_root.txt"

rem 2) 从 Anaconda Prompt 启动时，可从 CONDA_EXE 推出安装位置
if not defined CONDA_ROOT if defined CONDA_EXE for %%I in ("%CONDA_EXE%\..\..") do set "CONDA_ROOT=%%~fI"

rem 3) 常见安装位置
if not defined CONDA_ROOT for %%D in (
  "%USERPROFILE%\anaconda3" "%USERPROFILE%\miniconda3"
  "%ProgramData%\anaconda3" "%ProgramData%\miniconda3"
  "%LOCALAPPDATA%\anaconda3" "%LOCALAPPDATA%\miniconda3"
  "C:\anaconda3" "D:\anaconda3" "E:\anaconda3"
  "C:\miniconda3" "D:\miniconda3" "E:\miniconda3"
) do if not defined CONDA_ROOT if exist "%%~D\condabin\conda.bat" set "CONDA_ROOT=%%~D"

if not defined CONDA_ROOT (
  echo [错误] 找不到 Anaconda 安装位置。
  echo   请在 Anaconda Prompt 里运行：where conda
  echo   把输出路径中 \condabin 或 \Scripts 之前的部分，
  echo   例如 D:\Software\anaconda3 ，写进本文件夹的 conda_root.txt 里再试。
  exit /b 1
)
if not exist "%CONDA_ROOT%\condabin\conda.bat" (
  echo [错误] %CONDA_ROOT% 不是有效的 Anaconda 目录，请检查 conda_root.txt
  exit /b 1
)
if /i "%~1"=="root" exit /b 0

call "%CONDA_ROOT%\Scripts\activate.bat" %ENV_NAME% >nul 2>&1
python -c "import cv2, mss, PIL, numpy" >nul 2>&1
if errorlevel 1 (
  echo [错误] 环境 %ENV_NAME% 不存在或依赖没装好，请先双击「创建环境.bat」。
  exit /b 1
)
exit /b 0
