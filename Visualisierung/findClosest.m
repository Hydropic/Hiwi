function closest_idx = findClosest(array, values)
    [~, closest_idx] = min(abs(bsxfun(@minus, array, values)), [], 1);
end