def aslist(x):
    if x == "":
        return []
    else:
        return x.split(';')

def asdict(x):
    dict = {}
    for str in aslist(x):
        pair = str.split("@")
        dict[pair[0]] = pair[1]

    return dict
