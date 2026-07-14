// Copyright Epic Games, Inc. All Rights Reserved.

#include "MUMT_Sim.h"
#include "Modules/ModuleManager.h"
#include "State/MumtCommandArbiterV2.h"

// The command RESOLVER is a PRODUCTION facility and is bound for the whole lifetime of this module.
//
// That is the point of Phase A. The resolver is what makes CopyToJSBSim's consume boundary the single
// place where the final command block is decided. If it were bound only by tests, production would still
// be running the old "whichever writer happened to go last wins" behaviour, and every arbiter test would
// be proving something about a code path that never actually runs.
//
// Binding it changes nothing about how anything flies: every aircraft defaults to
// ECommandMode::LegacyOrManual, in which the resolved block equals the legacy block in all 29 consumed
// fields. FormationControlV2 is NOT activated here, and cannot be reached by config, Blueprint default,
// or a UDP packet -- only by an explicit test API call.
class FMumtSimModule : public FDefaultGameModuleImpl
{
public:
	virtual void StartupModule() override
	{
		FDefaultGameModuleImpl::StartupModule();
		MumtCommandArbiterV2::SetEnabled(true);
	}

	virtual void ShutdownModule() override
	{
		MumtCommandArbiterV2::SetEnabled(false);
		FDefaultGameModuleImpl::ShutdownModule();
	}
};

IMPLEMENT_PRIMARY_GAME_MODULE( FMumtSimModule, MUMT_Sim, "MUMT_Sim" );
