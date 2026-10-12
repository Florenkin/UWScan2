@echo off
setlocal

set "VS_ROOT=C:\Program Files\Microsoft Visual Studio\18\Community"
set "CMAKE_EXE=%VS_ROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NINJA_EXE=%VS_ROOT%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
set "REPO_ROOT=C:\Users\AAA\Desktop\UWScan2"
set "BUILD_DIR=%REPO_ROOT%\build-uwscan3d-algorithms"

call "%VS_ROOT%\VC\Auxiliary\Build\vcvars64.bat" || exit /b 1

"%CMAKE_EXE%" -G Ninja ^
  -S "%REPO_ROOT%\dll_project" ^
  -B "%BUILD_DIR%" ^
  -DCMAKE_MAKE_PROGRAM="%NINJA_EXE%" ^
  -DCMAKE_BUILD_TYPE=Release || exit /b 1

"%CMAKE_EXE%" --build "%BUILD_DIR%" || exit /b 1

endlocal
