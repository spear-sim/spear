To compile InteriorSim on windows, there needs to be some changes made;

Thirdparty libs 
- Need to add  "-DCMAKE_CXX_FLAGS='/bigobj'" (https://stackoverflow.com/questions/69903253/cmake-bigobj-no-such-file-or-directory) to cmake command for rbdl build.
- For compile error C2059, fix by defining NOMINMAX for windows platform only. https://github.com/google/flatbuffers/issues/105, https://stackoverflow.com/questions/11544073/how-do-i-deal-with-the-max-macro-in-windows-h-colliding-with-max-in-std. Add the following to this file -  addons/urdfreader/urdfreader.cc
  - #ifdef _MSC_VER
        #define NOMINMAX
    #endif
- Run `create_symbolic_links.py` in an anaconda prompt in Admin mode.


RobotProject
- Compile with just RobotSim. No issues with rbdl.
- Compile with  RobotSim + CoreUtils. Errors with assert statements in rbdl (2>C:\Users\Rachith\Documents\Unreal Projects\interiorsim\code\unreal_projects\RobotProject\Plugins\RobotSim\ThirdParty\rbdl\include\rbdl/Body.h(115): error C3861: 'assert': identifier not found
) and few errors with dllimport function not allowed in Assert.h::524.
	- remove COREUTILS_API from AssertUsedWrapper class to not export it as a DLL. This eliminates the dllimport errors.
	- to handle assert errors, change the file name of Assert.h and Assert.cpp as it coincides with STL's <assert.h> and causes compiler errors.
- remove dependency on virtual world manager plugin as it is causing compile issues.(temp solution)
- Once compile issues above are resolved, could not load dll error pops up while cooking. 'C:/Users/Rachith/Documents/Unreal Projects/interiorsim/code/unreal_projects/RobotProject/Plugins/CoreUtils/Binaries/Win64/UE4Editor-CoreUtils.dll' because the file couldn't be loaded by the OS`.
  - To resolve this, change loadingphase of all plugins to "None". Not sure why, but this works on windows. After the first cook is successfull, change loadingphase back to original or all to 'Default'.
- After resolving all compile errors above, package builds successfully. However, while executing it, error pops up saying could not load yaml-cpp.dll.
  - For this error, include both of these in coreutils.build.cs
      - PublicDelayLoadDLLs.Add(Path.Combine(ModuleDirectory, "..", "ThirdParty", "yaml-cpp", "build", "Release", "yaml-cpp.dll"));
      - RuntimeDependencies.Add("$(TargetOutputDir)/yaml-cpp.dll", Path.Combine(ModuleDirectory, "..", "ThirdParty", "yaml-cpp", "build", "Release", "yaml-cpp.dll"));
