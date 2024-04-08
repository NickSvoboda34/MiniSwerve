def inputModulus(input, min, max):
    mod = max - min

    numMax = int((input - min) / mod)
    input -= numMax * mod

    numMin = int((input - max) / mod)
    input -= numMin * mod

    return input