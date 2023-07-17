from typing import List, Union

class MMwaveCommand:
    _command_types = []
    _extra_type = None
    _num_optional = 0
    _name = "base"

    def __init__(self, command_parameters:List[Union[int, float]]):
        self.command_parameters = command_parameters
    
    def __str__(self):
        return self._name + " " + " ".join([str(x) for x in self.command_parameters])
    
    @classmethod
    def _parse_args(cls, cmd_args:str) -> List[Union[int, float]]:
        if len(cmd_args) == 0 and len(cls._command_types) - cls._num_optional == 0:
            return []

        parts = cmd_args.split(" ")
        if len(parts) < len(cls._command_types) - cls._num_optional:
            raise ValueError(f"Not enough arguments for command {cls._name}: {cmd_args}")
        if len(parts) > len(cls._command_types) and cls._extra_type is None:
            raise ValueError(f"Too many arguments for command {cls._name}: {cmd_args}")
        try:
            args = [cls._command_types[i](parts[i]) for i in range(len(cls._command_types))]
            if cls._extra_type is not None:
                num_extra = len(parts) - len(cls._command_types)
                args.extend([cls._extra_type(parts[i + len(cls._command_types)]) for i in range(num_extra)])
            return args
        except ValueError:
            raise ValueError(f"Invalid string for command {cls._name}: {cmd_args}")
    
    @classmethod
    def parse(cls, args):
        return cls(cls._parse_args(args))

