extends Node

func _enter_tree() -> void:
    if ClassDB.class_exists(&"EvolvePhysicsServer"):
        PhysicsServer2DManager.register_server(
            "EvolvePhysics",
            Callable(self, "_create_server")
        )
        print("[EvolvePhysics] ✓ Rapier2D physics server registered")
    else:
        print("[EvolvePhysics] ⚠ Extension not loaded")

func _create_server() -> PhysicsServer2D:
    return EvolvePhysicsServer.new()
