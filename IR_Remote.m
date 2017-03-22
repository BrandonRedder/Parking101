function [word] = IR_Remote(hex_command)
    command = hexToBinaryVector(hex_command,8);
    word = binaryVectorToHex([command(end:-1:1), ~command(end:-1:1)]);
end

