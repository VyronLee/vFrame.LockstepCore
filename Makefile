SolutionFile = vFrame.Lockstep.Core.sln

Compiler = MSBuild /t:Build /p:Platform=Any\ CPU

ConfigRelease = /p:Configuration=Release

BuildDir = Build
BuildReleaseDir = $(BuildDir)/Release
ReleaseRuntimeDir = $(BuildReleaseDir)/Runtime

OutputDir = Output
RuntimeOutputDir = $(OutputDir)/Runtime

RuntimeAssembly = vFrame.Lockstep.Core.dll

all: runtime output_runtime

clean:
	rm -rf $(OutputDir)/Runtime/*.dll
	rm -rf $(BuildDir)

runtime:
	$(Compiler) $(ConfigRelease) $(SolutionFile)

output_runtime:
	mkdir -p $(RuntimeOutputDir)
	cp -rf $(ReleaseRuntimeDir)/$(RuntimeAssembly) $(RuntimeOutputDir)/$(RuntimeAssembly)

.PHONY:
	clean editor runtime output_editor output_runtime

