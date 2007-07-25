function funcs = defaultDistFuncs()

funcs.transformed_distance = @distanceFunc;
funcs.max_distance_to_rectangle = @maxToRect;
funcs.min_distance_to_rectangle = @minToRect;
funcs.new_distance = @newDist;