#!/usr/bin/env python3
import csv
from dataclasses import dataclass
import re
import os

CSV_FILE = "ti_mmwave_cfg_format.csv"
HEADER_FILE = "mmwave_commands_header.py"
FOOTER_FILE = "mmwave_commands_footer.py"

def get_file(filename):
    with open(filename) as f:
        return f.read()

class ParameterInferrer:
    filename = "user_input.txt"
    @dataclass
    class InferredParameter:
        info: str
        usage: str
        parent_command: str
        offset: int
        name: str=""
        list_type: str=""
        option_type: str=""
        optional: bool=False
        extra: bool=False

        def get_option_type_key(self):
            return f"{self.parent_command}.{self.offset}.option_type"
        
        def get_list_type_key(self):
            return f"{self.parent_command}.{self.offset}.list_type"
        
        def get_name_key(self):
            return f"{self.parent_command}.{self.offset}.name"

        def get_extra_key(self):
            return f"{self.parent_command}.{self.offset}.extra"
        
        def get_optional_key(self):
            return f"{self.parent_command}.{self.offset}.optional"
    
    def __init__(self):
        self.data = {}
        if not os.path.exists(self.filename):
            open(self.filename, "w").close()
        with open(self.filename) as f:
            for line in f.readlines():
                if line.strip() == "":
                    continue
                key, value = line.split("=", 1)
                self.data[key.strip()] = value.strip()
    
    def infer_or_ask_parameter(self, parameter_info, parameter_usage, parent_command, offset)->InferredParameter:
        inferred = self.InferredParameter(
            info=parameter_info,
            usage=parameter_usage,
            parent_command=parent_command,
            offset=offset,
        )
        self.infer_or_ask_parameter_name(inferred)
        self.infer_or_ask_parameter_type(inferred)
        return inferred
    
    def infer_or_ask_parameter_name(self, parameter:InferredParameter) -> None:
        if self.data.get(parameter.get_name_key()) is not None:
            parameter.name = self.data[parameter.get_name_key()]
            return
        matches = re.search(r"<([a-zA-Z][a-z0-9A-Z]*)>", parameter.info)
        if matches is not None and len(matches.groups()) == 1:
            parameter.name = matches.groups()[0]
            return
        print("=================NAME====================")
        print(parameter.info)
        suggested_name = self.make_var_name(parameter.info)
        answer = input(f"Enter variable name (blank for {suggested_name}):")
        name_to_go_with = answer
        if answer == "":
            name_to_go_with = suggested_name
        elif answer.isnumeric():
            name_to_go_with =  self.make_var_name(parameter.info, num_words=int(answer))
        parameter.name = name_to_go_with
        self.data[parameter.get_name_key()] = name_to_go_with


    def infer_or_ask_parameter_type(self, parameter: InferredParameter) -> None:
        if self.data.get(parameter.get_list_type_key()) is not None:
            parameter.list_type = self.data[parameter.get_list_type_key()]
            parameter.option_type = self.data[parameter.get_option_type_key()]
            parameter.optional = self.data[parameter.get_optional_key()] == "True"
            parameter.extra = self.data[parameter.get_extra_key()] == "True"
            return
        match = re.search(r"type|number of|flag|1 ?(-|:)|index|idx|mask|set to|identifier", parameter.info, re.IGNORECASE | re.MULTILINE)
        if match is not None:
            parameter.list_type = parameter.option_type = "int"
            return
        
        if re.match(r"(only value|set to|mode|option)", parameter.usage, re.IGNORECASE| re.MULTILINE)is not None:
            parameter.list_type = parameter.option_type = "int"
            return
            
        if "float" in parameter.info.lower():
            parameter.list_type = parameter.option_type = "float"
            return
        print("=================TYPE====================")
        print(parameter.info)
        answer = input("What type is this parameter? ")
        if answer == "i":
            parameter.list_type = parameter.option_type = "int"
        if answer == "f":
            parameter.list_type = parameter.option_type = "float"
        if answer == "o":
            parameter.list_type = "int"
            parameter.option_type = "Union[int, None]"
            parameter.optional = True
        if answer == "e":
            parameter.extra = True
            parameter.list_type = "float"
            parameter.option_type = "List[float]"
        
        self.data[parameter.get_list_type_key()] = parameter.list_type
        self.data[parameter.get_option_type_key()] = parameter.option_type
        self.data[parameter.get_optional_key()] = parameter.optional
        self.data[parameter.get_extra_key()] = parameter.extra


    def make_var_name(self, parameter_info, num_words=3):
        first_line = parameter_info.split("\n")[0]
        words = re.findall(r"[a-zA-Z0-9]+", first_line)
        first_word = words[0]
        num_words = min(num_words, len(words))
        if words[0].upper() != words[0]:
            first_word = words[0].lower()
        name = first_word + "".join([i.capitalize() for i in words[1:num_words]])
        
        return name 

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        with open(self.filename, "w") as f:
            for key, value in self.data.items():
                f.write(f"{key}={value}\n")


class CommandClassBuilder:
    def __init__(self, command_name: str, command_details:str, inferrer:ParameterInferrer):
        self.name = command_name
        self.details = command_details
        self.parameters = []
        self.num_optional = 0
        self.extra_type = None
        self.parameter_inferrer = inferrer
    
    def make_class(self):
        optional_part = ""
        if self.num_optional > 0:
            optional_part += f"\n    _num_optional = {self.num_optional}"
        if self.extra_type is not None:
            optional_part += f"\n    _extra_type = {self.extra_type}"

        return f"""
class {self.name}Command(MMwaveCommand):
    \"""{self.details}\"""

    _name = "{self.name}"
    _command_types = [{", ".join([p.list_type for p in self.parameters])}]{optional_part}
{"".join([self.make_property(p) for p in self.parameters]) }
"""

    def make_property(self, parameter:ParameterInferrer.InferredParameter) -> str:
        offset = str(parameter.offset)
        if parameter.extra:
            offset = offset + ":"
        return f"""
    @property
    def {parameter.name}(self) -> {parameter.option_type}:
        \"""{parameter.info}

        usage: {parameter.usage}\"""
        return self.command_parameters[{offset}]

    @{parameter.name}.setter
    def {parameter.name}(self, value: {parameter.option_type}) -> None:
        self.command_parameters[{offset}] = value
"""

    def add_parameter(self, parameter_info: str, parameter_usage:str) -> None:
        parameter = self.parameter_inferrer.infer_or_ask_parameter(parameter_info, parameter_usage, self.name, len(self.parameters))
        if parameter.optional:
            self.num_optional += 1
        if parameter.extra:
            self.extra_type = parameter.list_type
        self.parameters.append(parameter)


def make_command_classes():
    command_list = []

    with open(CSV_FILE, 'r') as csvfile:
        with ParameterInferrer() as inferrer:
            reader = csv.DictReader(csvfile)
            current_command = None
            for row in reader:
                if row["Configuration command"] != "":
                    if current_command is not None:
                        command_list.append(current_command)
                    current_command = CommandClassBuilder(row["Configuration command"], row["Command details"].replace("\n", " "), inferrer)
                if row["Command Parameters"] != "":
                    current_command.add_parameter(row["Command Parameters"].replace("\n", " "), row["Parameter Usage"].replace("\n", " "))
        
    return command_list

def make_parse_table(commands: CommandClassBuilder):
    parse_table = ",\n    ".join([f'"{c.name}": {c.name}Command.parse' for c in commands])
    return f"""
_PARSE_TABLE = {{
    {parse_table}
}}

"""

if __name__ == '__main__':
    classes = make_command_classes()
    with open("mmwave_commands.py", "w") as f:
        f.write(get_file(HEADER_FILE))
        for c in classes:
            f.write(c.make_class())
        f.write(make_parse_table(classes))
        f.write(get_file(FOOTER_FILE))
