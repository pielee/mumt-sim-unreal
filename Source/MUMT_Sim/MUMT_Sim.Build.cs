// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class MUMT_Sim : ModuleRules
{
	public MUMT_Sim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "JSBSimFlightDynamicsModel", "Json", "JsonUtilities", "Sockets", "Networking" });

		PrivateDependencyModuleNames.AddRange(new string[] {  });

		// Editor-only: the dev/automation live-snapshot test (WITH_EDITOR && WITH_DEV_AUTOMATION_TESTS)
		// drives PIE via UnrealEd latent commands. Excluded from non-editor (game/shipping) builds.
		if (Target.bBuildEditor)
		{
			PrivateDependencyModuleNames.Add("UnrealEd");
		}

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
