classdef ODQueue
    % A general queue datastructure for implementing the OnDeck queue

    properties
        list double
    end

    methods
        function ODQueue = ODQueue()
            ODQueue.list = double.empty;
        end

        function ODQueue = enqueue(ODQueue, elem)
            % Add an element to the queue
            ODQueue.list(end + 1, :) = elem;
        end

        function [ODQueue, out] = dequeue(ODQueue)
            % Pop the nxt element from the queue
            if isempty(ODQueue.list)
                out = [];
            else
                out = ODQueue.list(1, :);
                ODQueue.list(1, :) = [];
            end
        end

        function ODQueue = sort(ODQueue)
            % Sort the onDeck queue
            ODQueue.list = sortrows(ODQueue.list, 2);
        end
    end
end