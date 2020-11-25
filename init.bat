@echo off
:: allow expansion of variables before access
setlocal enabledelayedexpansion

set sourced=""
set cwd="%cd%"

if "%pythonpath%" == "" (
	echo "Environment not yet initialised."
	set current_path=""
	setx pythonpath "%cd%"
	echo "Environment set with: %cd%"
) else (
	echo "----------------------------"
	echo "Test if current pythonpath contains environment:"
	FOR /F %%p IN ("%pythonpath:;=";"%") do (
		echo "	# Pythonpath-Entry: %%~p"
		echo "	# Pythonpath-Candidate: %cwd%"
		if "%%~p" == "%cwd%" (
			echo "Found current working directory in path"
			set sourced="true"
		)
	)
	if !sourced! == "" (
		echo "--> Environment not yet configured."
		echo "----> Current pythonpath: %pythonpath%"
		echo "----> Current directory: %cd%"
		set current_path=%pythonpath%
		set new_path=%cd%;!current_path:"=!
		setx pythonpath "!new_path!"
		echo "--> Environment set with: !new_path!"
	) else (
		echo "-->Environment already configured."
	)
)
echo "----------------------------"
refreshenv


