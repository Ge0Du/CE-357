import sys

def parse_macros(def_file):
    macros = {}
    with open(def_file, "r") as f:
        in_components = False
        for line in f:
            if line.startswith("COMPONENTS"):
                in_components = True
            elif line.startswith("END COMPONENTS"):
                break
            elif in_components and line.strip().endswith(";"):
                parts = line.strip().split()
                if len(parts) >= 6:
                    name = parts[1]
                    x = int(parts[3])
                    y = int(parts[4])
                    orient = parts[5]
                    macros[name] = (x, y, orient)
    return macros

if len(sys.argv) != 3:
    print("Usage: python compare_macro_positions.py original.def macro_placed.def")
    sys.exit(1)

original = parse_macros(sys.argv[1])
placed = parse_macros(sys.argv[2])

print(f"{'Macro':<25} {'Orig (x,y)':<20} {'New (x,y)':<20} {'Moved?'}")
print("="*70)
for name in original:
    if name in placed:
        ox, oy, _ = original[name]
        px, py, _ = placed[name]
        moved = "YES" if (ox != px or oy != py) else "NO"
        print(f"{name:<25} ({ox},{oy})     ({px},{py})     {moved}")
    else:
        print(f"{name:<25} Not found in placed.def")
