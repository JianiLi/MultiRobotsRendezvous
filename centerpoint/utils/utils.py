import math
import numpy as np
import random
from shapely.geometry import Point
from shapely.geometry import MultiPoint


def findKthLargest(nums, k):
    '''
    O(n) time to find the k-th largest number in an array
    '''
    pivot = random.choice(nums)
    nums1, nums2 = [], []
    for num in nums:
        if num > pivot:
            nums1.append(num)
        elif num < pivot:
            nums2.append(num)
    if k <= len(nums1):
        return findKthLargest(nums1, k)
    if k > len(nums) - len(nums2):
        return findKthLargest(nums2, k - (len(nums) - len(nums2)))
    return pivot



