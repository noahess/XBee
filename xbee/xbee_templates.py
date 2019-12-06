import struct
import numpy as np


class Templates:
    def __init__(self):
        pass

    """
    Templates are derived from XBee documentation using this format:
    
        b'API ID': [
            "API ID String Name",
            ["Field 1 Name", Field_1_Start, Field_1_Length, "field_1_datatype"],
            ["Field 2 Name", Field_2_Start, Field_2_Length, "field_2_datatype"],
            ["Field 3 Name", Field_3_Start, Field_3_Length, "field_3_datatype"],
            ...
            ["Field n-1 Name", Field_n-1_Start, Field_n-1_Length, "field_n-1_datatype"],
            ["Field n Name", Field_n_Start, Field_n_Length, "field_n_datatype", "optional if Field n optional"]],
            
    Field start is from the XBee documentation
    Field length is length in bytes. If variable length, set to -1.
    Field data types follow:
        "uint8_t" - one byte, unsigned; converted to Python int
        "uint16_t" - two bytes, unsigned; converted to Python int
        "uint64_t" - eight bytes, unsigned; converted to Python int
        "str" - any length; converted to or from Python string
        "bytes" - any length > 1; converted to or from Python bytes
    The optional parameter can be included for the last field if the field is optional. Do not include if field is not
        optional.
                
    """

    templates = {
        b'\x08': [
            "AT Command Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["AT Command", 5, 2, "str"],
            ["Parameter Value", 7, -1, "bytes", "optional"]],
        b'\x09': [
            "AT Command Queue Parameter Value Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["AT Command", 5, 2, "str"],
            ["Parameter Value", 7, -1, "optional"]],
        b'\x10': [
            "Transmit Request Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["Destination Address", 5, 8, "uint64_t"],
            ["Reserved", 13, 2, "uint16_t"],
            ["Broadcast Radius", 15, 1, "uint8_t"],
            ["Transmit Options", 16, 1, "uint8_t"],
            ["RF data", 17, -1, "bytes"]],
        b'\x11': [
            "Explicit Addressing Command Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["Destination Address", 5, 8, "uint64_t"],
            ["Reserved", 13, 2, "uint16_t"],
            ["Source endpoint", 15, 1, "uint8_t"],
            ["Destination endpoint", 16, 1, "uint8_t"],
            ["Cluster ID", 17, 2, "uint16_t"],
            ["Profile ID", 19, 2, "uint16_t"],
            ["Broadcast Radius", 21, 1, "uint8_t"],
            ["Transmit Options", 22, 1, "uint8_t"],
            ["RF data", 23, -1, "bytes"]],
        b'\x17': [
            "Remote AT Command Request Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["Destination Address", 5, 8, "uint64_t"],
            ["Reserved", 13, 2, "uint16_t"],
            ["Remote Command Options", 15, 1, "uint8_t"],
            ["AT command", 16, 2, "str"],
            ["Command Parameter", 18, -1, "bytes", "optional"]],
        b'\x88': [
            "AT Command Response Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["AT Command", 5, 2, "str"],
            ["Command Status", 7, 1, "uint8_t"],
            ["Command Data", 8, -1, "bytes", "optional"]],
        b'\x89': [
            "TX Status Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["Status", 5, 1, "uint8_t"]],
        b'\x8A': [
            "Modem Status Frame",
            ["Status", 4, 1, "uint8_t"]],
        b'\x8B': [
            "Transmit Status Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["Reserved", 5, 2, "uint16_t"],
            ["Transmit Retry Count", 7, 1, "uint8_t"],
            ["Delivery Status", 8, 1, "uint8_t"],
            ["Discovery Status", 9, 1, "uint8_t"]],
        b'\x90': [
            "Receive Packet Frame",
            ["Source Address", 4, 8, "uint64_t"],
            ["Reserved", 12, 2, "uint16_t"],
            ["Receive options", 14, 1, "uint8_t"],
            ["Received data", 15, -1, "bytes"]],
        b'\x91': [
            "Explicit Rx Indicator Frame",
            ["Source Address", 4, 8, "uint64_t"],
            ["Reserved", 12, 2, "uint16_t"],
            ["Source endpoint", 14, 1, "uint8_t"],
            ["Destination endpoint", 15, 1, "uint8_t"],
            ["Cluster ID", 16, 2, "uint16_t"],
            ["Profile ID", 18, 2, "uint16_t"],
            ["Receive options", 20, 1, "uint8_t"],
            ["Received data", 21, -1, "bytes"]],
        b'\x97': [
            "Remote Command Response Frame",
            ["Frame ID", 4, 1, "uint8_t"],
            ["Destination Address", 5, 8, "uint64_t"],
            ["Reserved", 13, 2, "uint16_t"],
            ["AT command", 15, 2, "str"],
            ["Command Status", 17, 1, "uint8_t"],
            ["Command Parameter", 18, -1, "bytes"]]
    }

    def can_process(self, message: bytes) -> bool:
        """
        Returns true if the API ID is known
        :param message: The message or first byte of message
        :type message: bytes
        :return: True if API ID is known
        :rtype: bool
        """
        return bytes([message[0]]) in self.templates

    def generate_struct_decode(self, message: bytes) -> tuple:
        """
        Generates Python struct string for a message to decode
        :param message: The incoming message to decode
        :type message: bytes
        :return: A tuple - (the struct string, the struct result mask)
        :rtype: tuple
        """
        assert self.can_process(message)                                    # Assert message is readable
        type_string = ">B"                                                  # First byte is the API ID
        mask_string = b"\x00"                                               # First struct result is the 0th argument

        template = self.templates[bytes([message[0]])]                      # Obtain template

        for component in range(1, len(template)):                           # Iterate over template fields
            component_start = template[component][1]                        # Offset of template field
            component_length = template[component][2]                       # Length in bytes of template field
            component_type = template[component][3]                         # Data type of template field
            is_optional = False                                             # Check if template field is optional
            if len(template[component]) >= 5:
                if template[component][4] == "optional":
                    is_optional = True

            if component_type == "bytes" or component_type == "str":        # Calculate length for bytes or string type
                byte_length = len(message) + 3 - component_start if (
                        is_optional or component_length == -1) else component_length
                if not (is_optional and byte_length == 0):                  # If data exists, include in string
                    type_string += str(byte_length) + "c"
                    mask_string += byte_length * bytes([component])
            elif component_type == "uint64_t":                              # 64 bit type
                type_string += "Q"
                mask_string += bytes([component])
            elif component_type == "uint16_t":                              # 16 bit type
                type_string += "H"
                mask_string += bytes([component])
            elif component_type == "uint8_t":                               # 8 bit type
                type_string += "B"
                mask_string += bytes([component])

        return type_string, mask_string

    def generate_struct_encode(self, args) -> str:
        """
        Generates Python struct string for a message to encode
        :param args: Message arguments
        :return: The corresponding struct string
        :rtype: str
        """
        template_type = bytes([args[0]])                                    # Obtain template type
        assert self.can_process(template_type)                              # Assert can continue with encoding
        type_string = ">B"                                                  # First element in string is API ID

        num_args = len(args)                                                # Calculate arg length

        template = self.templates[template_type]                            # Obtain template for type

        for component in range(1, num_args):                                # Iterate over template fields
            component_type = template[component][3]                         # Get template type

            if component_type == "bytes" or component_type == "str":        # Bytes or str type
                if isinstance(args[component], bytes):                      # If argument length > 1
                    byte_length = len(args[component])
                else:                                                       # If argument is a single element
                    byte_length = 1
                type_string += str(byte_length) + "c"
            elif component_type == "uint64_t":                              # 64 bit type
                type_string += "Q"
            elif component_type == "uint16_t":                              # 16 bit type
                type_string += "H"
            elif component_type == "uint8_t":                               # 8 bit type
                type_string += "B"

        return type_string

    # noinspection PyTypeChecker
    def unpack_message(self, message: bytes) -> list:
        """
        Generate frame array corresponding to input message
        :param message: The incoming message
        :type message: bytes
        :return: The array describing frame fields
        :rtype: list
        """
        type_string, mask_string = self.generate_struct_decode(message)     # Generate struct string and mask
        data_array = struct.unpack(type_string, message)                    # Unpack binary message
        template = self.templates[bytes([message[0]])]                      # Obtain template

        np_mask = np.frombuffer(mask_string, dtype=np.uint8)                # Mask struct output
        result_array = (np_mask.max() + 1) * [""]                           # Initialize resulting array

        for component_idx in range(np_mask.max() + 1):                      # Iterate over field count
            indexes = np.where(np_mask == component_idx)                    # Check mask for data locations
            values = [data_array[data_idx] for data_idx in indexes[0]]      # Get data from mask
            if len(values) > 1:                                             # If data is plural
                if template[component_idx][3] == "str":                     # Combine strings
                    result_array[component_idx] = b''.join(values).decode('utf8')
                elif template[component_idx][3] == "bytes":                 # Combine byte strings
                    result_array[component_idx] = b''.join(values)
            else:
                if template[component_idx][3] == "uint64_t":                # 64 bit type
                    result_array[component_idx] = int(values[0])
                elif template[component_idx][3] == "uint8_t":               # 8 bit type
                    result_array[component_idx] = int(values[0])
                else:                                                       # Other types
                    result_array[component_idx] = values[0]

        return result_array

    def explain_message(self, message: bytes) -> dict:
        """
        Explain input message
        :param message: The incoming message
        :type message: bytes
        :return: A dictionary explaining the input message
        :rtype: dict
        """
        data_array = self.unpack_message(message)                           # Unpack message
        template = self.templates[bytes([message[0]])]                      # Obtain template
        explanation = {"Name": template[0]}                                 # Name of template is first element
        for i in range(1, len(template)):                                   # Iterate over fields
            if len(data_array) > i:                                         # If field exists
                explanation[template[i][0]] = data_array[i]
            else:
                explanation[template[i][0]] = None                          # If field doesn't exist

        return explanation

    def pack_message(self, *args) -> bytes:
        """
        Convert arguments into a byte message
        :param args: The message arguments
        :return: The corresponding byte array
        :rtype: bytes
        """
        template_type = bytes([args[0]])                                    # Calculate template type
        assert self.can_process(template_type)                              # Assert can process this message

        struct_string = self.generate_struct_encode(args)                   # Obtain struct string
        out_args = []                                                       # Initialize struct arguments
        for arg_idx in range(len(args)):                                    # Iterate over arguments
            arg = args[arg_idx]
            this_type = self.templates[template_type][arg_idx][3]           # Get field type
            if isinstance(arg, str) or isinstance(arg, bytes):              # Encode multi-byte non-number fields
                for sub_arg in arg:
                    if isinstance(arg, str):
                        out_args.append(sub_arg.encode('ascii'))            # String encode
                    else:
                        out_args.append(bytes([sub_arg]))                   # Byte encode
            elif isinstance(arg, int) and this_type == "bytes":
                out_args.append(bytes([arg]))                               # Convert ints to bytes if required
            else:
                out_args.append(arg)                                        # Pass all other args through
        return struct.pack(struct_string, *out_args)                        # Return packed string
