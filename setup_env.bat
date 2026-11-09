@echo off
setlocal enabledelayedexpansion
chcp 65001 >nul
cd /d "%~dp0"

echo ==============================================================
echo    TURRET FCS // OTOMATIK ORTAM KURULUMU (SETUP ENV)
echo ==============================================================
echo.

rem ------------------------------------------------------------------
rem 1) Uygun Python bul: py -3.12/3.11/3.10, sonra PATH'teki python.
rem ------------------------------------------------------------------
set "PY_CMD="
call :try_py 3.12
call :try_py 3.11
call :try_py 3.10
if not defined PY_CMD (
    python -c "import sys; raise SystemExit(0 if (3,10) <= sys.version_info[:2] <= (3,12) else 1)" >nul 2>nul
    if not errorlevel 1 set "PY_CMD=python"
)

if not defined PY_CMD goto :no_python

echo [OK] Python bulundu:
%PY_CMD% -c "import sys; print('    ', sys.version.split()[0], sys.executable)"

rem ------------------------------------------------------------------
rem 2) Sanal ortam: yoksa veya bozuksa yeniden kur.
rem ------------------------------------------------------------------
set "VENV_PY=venv\Scripts\python.exe"
set "NEED_VENV=yes"
if not exist "%VENV_PY%" goto :build_venv

"%VENV_PY%" -c "import sys; raise SystemExit(0 if (3,10) <= sys.version_info[:2] <= (3,12) else 1)" >nul 2>nul
if errorlevel 1 goto :broken_venv

set "NEED_VENV=no"
echo [INFO] Mevcut venv kullanilabilir.
goto :have_venv

:broken_venv
echo [WARN] Mevcut venv bozuk veya farkli Python surumune bagli; yeniden kuruluyor.
goto :build_venv

:build_venv
echo [1/3] Sanal ortam olusturuluyor (venv)...
if exist "venv" rmdir /s /q "venv"
%PY_CMD% -m venv venv
if errorlevel 1 goto :venv_failed
echo [OK] venv olusturuldu.
goto :have_venv

:have_venv

rem ------------------------------------------------------------------
rem 3) pip
rem ------------------------------------------------------------------
echo.
echo [2/3] pip guncelleniyor...
"%VENV_PY%" -m pip install --upgrade pip
if errorlevel 1 goto :pip_failed

rem ------------------------------------------------------------------
rem 4) Bagimliliklar
rem    PyTorch CPU wheel'i ayri indeksten kurulur; boylece 2.5 GB'lik CUDA
rem    indirmesi yapilmaz.
rem ------------------------------------------------------------------
echo.
echo [3/3] Bagimliliklar ve paket kuruluyor (pyproject.toml, gelistirme modu)...
"%VENV_PY%" -m pip install -e ".[test]" --extra-index-url https://download.pytorch.org/whl/cpu
if errorlevel 1 goto :deps_failed

echo.
echo [Dogrulama] Temel importlar kontrol ediliyor...
"%VENV_PY%" -c "import numpy, cv2, torch, ultralytics, serial, turret_fcs, turret_basket; print('[OK] numpy', numpy.__version__, 'opencv', cv2.__version__, 'torch', torch.__version__)"
if errorlevel 1 goto :verify_failed

rem ------------------------------------------------------------------
rem 5) Webots runtime.ini dosyalarini yerel mutlak yola gore yapilandir
rem ------------------------------------------------------------------
echo.
echo [Webots] Webots kontrolculeri icin runtime.ini yapilandiriliyor...
set "VENV_ABS_PY=%~dp0venv\Scripts\python.exe"
set "VENV_ABS_PY=!VENV_ABS_PY:\=/!"
for %%C in (balloon_mover webots_turret_basket webots_turret_fcs) do (
    if exist "webots\controllers\%%C" (
        (
            echo [python]
            echo COMMAND = !VENV_ABS_PY!
            echo.
            echo [environment variables]
            echo PYTHONUNBUFFERED = 1
        ) > "webots\controllers\%%C\runtime.ini"
    )
)
echo [OK] Webots runtime.ini dosyalari hazirlandi (!VENV_ABS_PY!).

echo.
echo ==============================================================
echo  [TAMAMLANDI] Tum kutuphaneler basariyla kuruldu!
echo  Simdi 'turret_fcs\main.bat' veya 'turret_basket\main.bat' ile sistemi calistirabilirsiniz.
echo  Testler icin: venv\Scripts\python.exe -m pytest -q
echo ==============================================================
goto :done

:no_python
echo [HATA] Python 3.10 - 3.12 bulunamadi.
echo        https://www.python.org/downloads/ adresinden kurun ve
echo        kurulumda "Add python.exe to PATH" secenegini isaretleyin.
goto :fail

:venv_failed
echo [HATA] venv olusturulamadi.
goto :fail

:pip_failed
echo [HATA] pip guncellenemedi. Internet baglantisini kontrol edin.
goto :fail

:deps_failed
echo [HATA] Bagimliliklar kurulamadi.
goto :fail

:verify_failed
echo [HATA] Kurulum dogrulanamadi.
goto :fail

:fail
if not defined CI pause
endlocal
exit /b 1

:done
if not defined CI pause
endlocal
exit /b 0

:try_py
if defined PY_CMD exit /b 0
py -%~1 -c "import sys" >nul 2>nul
if not errorlevel 1 set "PY_CMD=py -%~1"
exit /b 0
