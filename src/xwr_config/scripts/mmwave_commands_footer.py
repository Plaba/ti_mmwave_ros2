def parse_command(command:str) -> MMwaveCommand:
    parts = command.split(" ", 1)
    name = parts[0]
    if len(parts) == 1:
        args = ""
    else:
        args = parts[1]
    return _PARSE_TABLE[name](args)

def parse_cfg_file(contents:str) -> List[MMwaveCommand]:
    commands = []
    for line in contents.split("\n"):
        if line.startswith("%") or line.strip() == "":
            continue
        commands.append(parse_command(line))
    return commands
