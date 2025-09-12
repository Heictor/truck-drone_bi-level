classdef priority_queue < handle
% A simple priority queue implementation for the A* algorithm.
% It is a handle class, so its properties can be modified by its methods.

    properties
        list % A cell array to store items {item, priority}
    end
    
    methods
        % Constructor to initialize the queue
        function self = priority_queue()
            self.list = cell(0);
        end
        
        % Method to add an item with a given priority
        function insert(self, item, priority)
            self.list{end+1} = {item, priority};
        end
        
        % Method to extract the item with the lowest priority value
        function item = extract_min(self)
            if self.isempty()
                item = [];
                return;
            end
            priorities = cellfun(@(x) x{2}, self.list);
            [~, min_idx] = min(priorities);
            item = self.list{min_idx}{1};
            self.list(min_idx) = [];
        end
        
        % Method to check if the queue is empty
        function is_empty = isempty(self)
            is_empty = isempty(self.list);
        end
        
        % Method to check if an item is already in the queue
        function does_contain = contains(self, item)
            does_contain = any(cellfun(@(x) isequal(x{1}, item), self.list));
        end
    end
end