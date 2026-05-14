"""Quick test: spawn a table and a cup, then exit."""

from spawn_objects import WorldSpawner


def main():
    spawner = WorldSpawner(world_name="empty")

    print("Spawning table...")
    ok = spawner.spawn_table(x=0.5, y=0.0, top_height=0.4)
    print("  Table:", "OK" if ok else "FAIL")

    print("Spawning cup...")
    ok = spawner.spawn_cup(x=0.5, y=0.0, table_top_height=0.4)
    print("  Cup:", "OK" if ok else "FAIL")


if __name__ == "__main__":
    main()