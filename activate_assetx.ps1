# AssetX USD 开发环境快速启动

$ErrorActionPreference = "Stop"

Write-Host "🚀 AssetX USD开发环境快速启动" -ForegroundColor Cyan
Write-Host "=" * 50 -ForegroundColor Cyan

# 检查是否在正确目录
if (-not (Test-Path "assetx")) {
    Write-Host "❌ 错误: 请在AssetX项目根目录运行此脚本" -ForegroundColor Red
    Write-Host "   当前目录: $(Get-Location)" -ForegroundColor Yellow
    Write-Host "   应该包含: assetx 文件夹" -ForegroundColor Yellow
    Read-Host "按回车键退出"
    exit 1
}

# 检查conda是否可用
try {
    $condaVersion = conda --version
    Write-Host "✅ Conda可用: $condaVersion" -ForegroundColor Green
} catch {
    Write-Host "❌ Conda未安装或未在PATH中" -ForegroundColor Red
    Write-Host "💡 请先安装Anaconda或Miniconda" -ForegroundColor Yellow
    Read-Host "按回车键退出"
    exit 1
}

# 检查assetx-usd环境是否存在
$envList = conda env list
if ($envList -notmatch "assetx-usd") {
    Write-Host "❌ assetx-usd 环境不存在" -ForegroundColor Red
    Write-Host "💡 请先运行以下命令创建环境:" -ForegroundColor Yellow
    Write-Host "   conda env create -f environment.yml" -ForegroundColor Cyan
    Read-Host "按回车键退出"
    exit 1
}

Write-Host "✅ 找到 assetx-usd 环境" -ForegroundColor Green

# 显示使用信息
Write-Host "`n🎯 常用命令:" -ForegroundColor Cyan
Write-Host "   测试USD功能:  python examples/simple_usd_test.py" -ForegroundColor White
Write-Host "   运行测试:     pytest tests/" -ForegroundColor White
Write-Host "   查看帮助:     python -m assetx.cli --help" -ForegroundColor White
Write-Host "   退出环境:     conda deactivate" -ForegroundColor White

Write-Host "`n🚀 正在启动AssetX USD开发环境..." -ForegroundColor Cyan
Write-Host "=" * 50 -ForegroundColor Cyan

# 激活环境并启动新的PowerShell会话
& powershell -NoExit -Command {
    conda activate assetx-usd
    
    # 检查激活是否成功
    if ($LASTEXITCODE -eq 0) {
        Write-Host "🎉 AssetX USD开发环境已就绪!" -ForegroundColor Green
        
        # 快速状态检查
        Write-Host "`n📊 环境状态:" -ForegroundColor Cyan
        
        # 检查Python
        try {
            $pythonVersion = python --version
            Write-Host "   Python: $pythonVersion" -ForegroundColor Green
        } catch {
            Write-Host "   Python: ❌ 检查失败" -ForegroundColor Red
        }
        
        # 检查USD
        try {
            $usdVersion = python -c "from pxr import Usd; print(f'USD版本: {Usd.GetVersion()}')"
            Write-Host "   USD: ✅ $usdVersion" -ForegroundColor Green
        } catch {
            Write-Host "   USD: ❌ 未安装或有问题" -ForegroundColor Red
        }
        
        # 检查AssetX
        try {
            python -c "from assetx import Asset; print('AssetX检查完成')" | Out-Null
            Write-Host "   AssetX: ✅ 可用" -ForegroundColor Green
        } catch {
            Write-Host "   AssetX: ❌ 未安装" -ForegroundColor Red
        }
        
        Write-Host "`n💡 使用 'conda deactivate' 退出环境" -ForegroundColor Yellow
        Write-Host "💡 输入 'cls' 清屏" -ForegroundColor Yellow
    } else {
        Write-Host "❌ 环境激活失败" -ForegroundColor Red
    }
}
