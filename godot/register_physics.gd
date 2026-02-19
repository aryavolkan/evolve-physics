extends Node

func _enter_tree() -> void:
	if ClassDB.class_exists(&"EvolvePhysicsServer"):
		PhysicsServer2DManager.register_server(
			"EvolvePhysics",
			func(): return EvolvePhysicsServer.new()
		)
		print("[EvolvePhysics] ✓ Rapier2D physics server registered")
	else:
		print("[EvolvePhysics] ⚠ Extension not loaded")
