@echo off
REM AssetX USD Development Environment Quick Start
REM Usage: Double-click to run or execute in command line

echo =======================================
echo   AssetX USD Development Environment
echo =======================================

REM Check if in correct directory
if not exist "assetx" (
    echo ERROR: Please run this script in AssetX project root directory
    echo Current directory: %CD%
    echo Should contain: assetx folder
    pause
    exit /b 1
)

REM Activate conda environment
echo Activating assetx-usd environment...
call conda activate assetx-usd
if errorlevel 1 (
    echo ERROR: Cannot activate assetx-usd environment
    echo Please create environment first:
    echo   conda env create -f environment.yml
    pause
    exit /b 1
)

REM Show environment info
echo.
echo SUCCESS: Environment activated!
echo.
echo Environment Status:
python --version
python -c "from pxr import Usd; print(f'USD Version: {Usd.GetVersion()}')" 2>nul || echo USD: Not installed
python -c "from assetx import Asset; print('AssetX: Available')" 2>nul || echo AssetX: Not installed

echo.
echo Common Commands:
echo   Test USD:     python examples/simple_usd_test.py
echo   Run tests:    pytest tests/
echo   CLI help:     python -m assetx.cli --help
echo   Exit env:     conda deactivate
echo.
echo Ready for AssetX USD development!
echo =======================================

REM Start PowerShell and keep in assetx-usd environment
powershell -NoExit -Command "Write-Host 'AssetX USD Environment Ready!' -ForegroundColor Green"
