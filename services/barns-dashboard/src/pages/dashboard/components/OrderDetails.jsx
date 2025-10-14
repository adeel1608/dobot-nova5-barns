import React, { useState, useEffect, useRef } from 'react';
import useStore from '../../../store';
import stop from '../../../assets/stop.png';
import coffee from '../../../assets/Coffee.png';
import lighting from '../../../assets/lighting.png';
import dots from '../../../assets/dots.png';
import progressing from '../../../assets/progressing.png';
import circledots from '../../../assets/circledots.png';
export default function OrderDetails() {
  const { orders, schedulerTasks, schedulerTaskStatus, schedulerStatusMessage, taskTimings, updateTaskTiming } = useStore(state => ({
    orders: state.orders,
    schedulerTasks: state.schedulerTasks || { Arm1: [], Arm2: [] },
    schedulerTaskStatus: state.schedulerTaskStatus || {},
    schedulerStatusMessage: state.schedulerStatusMessage || null,
    taskTimings: state.taskTimings || {},
    updateTaskTiming: state.updateTaskTiming
  }));
  const [showTaskInterface, setShowTaskInterface] = useState(true);
  const arm1ContainerRef = useRef(null);
  const arm2ContainerRef = useRef(null);
  const itemRefs = useRef({});
  
  // Track current time for live timers
  const [currentTime, setCurrentTime] = useState(Date.now());
  
  // Track previous in-progress tasks to detect when a new one starts
  const prevInProgressRef = useRef({ Arm1: null, Arm2: null });

  // Find the currently processing order
  const processingOrder = orders?.find(order => 
    order.status?.toUpperCase() === 'PROCESSING'
  );

  // Find the last completed/failed order if no processing order
  const lastFinishedOrder = !processingOrder ? orders
    ?.filter(order => ['COMPLETED', 'ERROR', 'STOPPED', 'CANCELLED'].includes(order.status?.toUpperCase()))
    ?.sort((a, b) => {
      const aTime = new Date(a.completed_at || a.started_at || a.created_at).getTime();
      const bTime = new Date(b.completed_at || b.started_at || b.created_at).getTime();
      return bTime - aTime; // Most recent first
    })[0] : null;

  // Use processing order if available, otherwise use last finished order
  const displayedOrder = processingOrder || lastFinishedOrder;
  const isCurrentOrder = !!processingOrder;
  
  // Determine if task state is frozen (order completed/failed/stopped)
  const isTasksFrozen = displayedOrder && ['COMPLETED', 'ERROR', 'STOPPED', 'CANCELLED'].includes(displayedOrder.status?.toUpperCase());

  // Show task interface if enabled, otherwise show idle message
  if (!showTaskInterface && !displayedOrder) {
    return (
      <div className=" rounded-lg border border-gray-200 flex flex-col h-full ">
        {/* Header - Responsive */}
        <div className="p-2 md:p-3  flex-shrink-0 flex  justify-between  p-4">
          <h2 className="text-base md:text-lg font-semibold text-gray-900">Current Order</h2>
          <div className="flex items-center space-x-2">
            <button 
              onClick={() => setShowTaskInterface(true)}
              className="px-3 py-1 bg-green-600 text-white text-xs rounded hover:bg-green-700 transition-colors"
            >
              Show Tasks
            </button>
            <button className="flex text-red bg-red-300 hover:bg-red-600 text-white" style={{ padding: '0.3rem', outline: 'none', }}>
              {/* Stop */}
              <img src={stop} alt="Refresh" className="w-6 h-6 cursor-pointer " />
           </button>
          </div>
        </div>

        {/* No Processing Order - Responsive */}
        <div className="flex-1 flex items-center justify-center text-gray-500 p-3 md:p-4">
          <div className="text-center">
            <img src={coffee} alt="Coffee" className="w-20 h-20 mx-auto mb-2 md:mb-3" />
            <p className="text-sm font-medium">No Order Processing</p>
            <p className="text-xs text-gray-400 mt-1">System is idle</p>
          </div>
        </div>
      </div>
    );
  }

  // Format order data for display (only if we have a processing order)
  const formatOrderForDisplay = (order) => {
    if (!order) return null;
    
    return {
      ...order,
      itemName: order.cups && order.cups.length > 0 
        ? order.cups.map(cup => `${cup.drink_type || cup.type} (${cup.cup_size || cup.size})`).join(', ')
        : 'Unknown Item',
      createdAt: order.created_at ? new Date(order.created_at).toLocaleString() : 'N/A',
      startedAt: order.started_at ? new Date(order.started_at).toLocaleString() : 'N/A'
    };
  };

  const displayOrder = displayedOrder ? formatOrderForDisplay(displayedOrder) : null;

  // Calculate cup-level progress
  const cupProgress = React.useMemo(() => {
    if (!displayedOrder || !displayedOrder.cups) return { completed: 0, total: 0, percentage: 0 };
    
    const totalCups = displayedOrder.cups.length;
    const allTasks = [...(schedulerTasks.Arm1 || []), ...(schedulerTasks.Arm2 || [])];
    
    // Group tasks by cup_id
    const cupTasks = {};
    allTasks.forEach(task => {
      if (!cupTasks[task.cup_id]) cupTasks[task.cup_id] = [];
      cupTasks[task.cup_id].push(task);
    });
    
    // Count completed cups (all tasks for that cup are completed)
    let completedCups = 0;
    Object.values(cupTasks).forEach(tasks => {
      const allCompleted = tasks.length > 0 && tasks.every(t => t.status === 'completed');
      if (allCompleted) completedCups++;
    });
    
    const percentage = totalCups > 0 ? Math.round((completedCups / totalCups) * 100) : 0;
    
    return { completed: completedCups, total: totalCups, percentage };
  }, [displayedOrder, schedulerTasks]);

  // Track task timing - start times and elapsed times (only for live orders)
  useEffect(() => {
    if (isTasksFrozen) return; // Don't update timings for frozen orders
    
    const allTasks = [...(schedulerTasks.Arm1 || []), ...(schedulerTasks.Arm2 || [])];
    const now = Date.now();
    
    allTasks.forEach((task) => {
      const taskKey = `${task.cup_id}:${task.action}`;
      const isInProgress = task.status === 'in_progress' || task.status === 'submitted';
      const isCompleted = task.status === 'completed' || task.status === 'failed' || task.status === 'cancelled';
      const existingTiming = taskTimings[taskKey];
      
      // Start timer when task becomes in progress
      if (isInProgress && !existingTiming) {
        updateTaskTiming(taskKey, { startTime: now, elapsedTime: null });
      }
      
      // Freeze timer when task completes
      if (isCompleted && existingTiming && existingTiming.elapsedTime === null) {
        const elapsed = now - existingTiming.startTime;
        updateTaskTiming(taskKey, { ...existingTiming, elapsedTime: elapsed });
      }
    });
  }, [schedulerTasks, taskTimings, updateTaskTiming, isTasksFrozen]);

  // Update current time every second for live timers (only for live orders)
  useEffect(() => {
    if (isTasksFrozen) return; // Don't update time for frozen orders
    
    const interval = setInterval(() => {
      setCurrentTime(Date.now());
    }, 1000);
    
    return () => clearInterval(interval);
  }, [isTasksFrozen]);

  // Auto-scroll to in-progress task when it changes (only for live orders)
  useEffect(() => {
    if (isTasksFrozen) return; // Don't auto-scroll for frozen orders
    
    try {
      const arm1 = schedulerTasks.Arm1 || [];
      const arm2 = schedulerTasks.Arm2 || [];

      // Find current in-progress tasks for each arm
      const arm1InProgress = arm1.find(t => t.status === 'in_progress' || t.status === 'submitted');
      const arm2InProgress = arm2.find(t => t.status === 'in_progress' || t.status === 'submitted');

      const arm1Key = arm1InProgress ? `${arm1InProgress.cup_id}:${arm1InProgress.action}` : null;
      const arm2Key = arm2InProgress ? `${arm2InProgress.cup_id}:${arm2InProgress.action}` : null;

      // Check if in-progress task changed for Arm1
      if (arm1Key && arm1Key !== prevInProgressRef.current.Arm1) {
        const container = arm1ContainerRef.current;
        const key = `Arm1:${arm1InProgress.cup_id}:${arm1InProgress.action}`;
        const el = itemRefs.current[key];
        
        if (container && el) {
          const top = el.offsetTop - (container.clientHeight / 2) + (el.clientHeight / 2);
          container.scrollTo({ top: Math.max(0, top), behavior: 'smooth' });
        }
        
        prevInProgressRef.current.Arm1 = arm1Key;
      } else if (!arm1Key && prevInProgressRef.current.Arm1) {
        // Reset if no in-progress task
        prevInProgressRef.current.Arm1 = null;
      }

      // Check if in-progress task changed for Arm2
      if (arm2Key && arm2Key !== prevInProgressRef.current.Arm2) {
        const container = arm2ContainerRef.current;
        const key = `Arm2:${arm2InProgress.cup_id}:${arm2InProgress.action}`;
        const el = itemRefs.current[key];
        
        if (container && el) {
          const top = el.offsetTop - (container.clientHeight / 2) + (el.clientHeight / 2);
          container.scrollTo({ top: Math.max(0, top), behavior: 'smooth' });
        }
        
        prevInProgressRef.current.Arm2 = arm2Key;
      } else if (!arm2Key && prevInProgressRef.current.Arm2) {
        // Reset if no in-progress task
        prevInProgressRef.current.Arm2 = null;
      }
    } catch (error) {
      // Silently handle any errors in auto-scroll
    }
  }, [schedulerTasks, isTasksFrozen]);

  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-200 flex flex-col h-full">
      {/* Header */}
      <div className={`p-2 sm:p-3 border-b flex-shrink-0 ${isCurrentOrder ? 'border-gray-200 bg-white' : 'border-gray-300 bg-gray-50'}`}>
        <div className="flex items-center justify-center space-x-1 sm:space-x-2 flex-wrap gap-y-1">
          {!isCurrentOrder && (
            <svg className="w-4 h-4 sm:w-5 sm:h-5 text-gray-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
            </svg>
          )}
          <h2 className={`text-base sm:text-lg font-semibold ${isCurrentOrder ? 'text-gray-900' : 'text-gray-600'}`}>
            {isCurrentOrder ? 'Current Order' : 'Last Order'}{displayedOrder ? `: ${displayedOrder.id}` : ''}
          </h2>
          {!isCurrentOrder && displayedOrder && (
            <span className={`text-[10px] sm:text-xs font-medium px-1.5 sm:px-2 py-0.5 sm:py-1 rounded-full ${
              displayedOrder.status?.toUpperCase() === 'COMPLETED' 
                ? 'bg-green-100 text-green-800' 
                : 'bg-red-100 text-red-800'
            }`}>
              {displayedOrder.status?.toUpperCase() === 'COMPLETED' ? 'Completed' : 'Failed'}
            </span>
          )}
        </div>
      </div>

      {/* Task Management Interface */}
      <div className="flex-1 p-2 sm:p-4 overflow-hidden flex flex-col">
        {/* Cup Progress Bar */}
        {cupProgress.total > 0 && (
          <div className="mb-3 sm:mb-4 flex-shrink-0">
            <div className="flex items-center justify-between mb-1">
              <span className="text-xs sm:text-sm font-medium text-gray-700">
                Cup Progress: {cupProgress.completed}/{cupProgress.total} Completed
              </span>
              <span className="text-xs font-semibold text-gray-600">{cupProgress.percentage}%</span>
            </div>
            <div className="w-full bg-gray-200 rounded-full h-2.5">
              <div 
                className={`h-2.5 rounded-full transition-all duration-300 ${
                  cupProgress.percentage === 100 ? 'bg-green-600' : 'bg-blue-600'
                }`}
                style={{ width: `${cupProgress.percentage}%` }}
              ></div>
            </div>
          </div>
        )}

        {schedulerStatusMessage && (
          <div className="mb-2 sm:mb-3 text-xs text-gray-500 flex-shrink-0">{schedulerStatusMessage}</div>
        )}

        {/* Task Columns */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 sm:gap-6 flex-1 overflow-hidden">
          {/* Left Column - Arm 1 Tasks (Live) */}
          <div className="flex flex-col h-full overflow-hidden">
            <h3 className="text-xs sm:text-sm font-semibold text-gray-900 mb-2 sm:mb-3 flex-shrink-0">Robot Arm 1</h3>
            <div ref={arm1ContainerRef} className="relative flex-1 overflow-y-auto pr-1 sm:pr-2 scroll-smooth pb-3 sm:pb-4">
              {schedulerTasks.Arm1.length === 0 ? (
                <div className="text-xs text-gray-400">No tasks yet.</div>
              ) : (
                <div className="space-y-2 sm:space-y-3">
                  {schedulerTasks.Arm1.map((t, idx) => (
                    <TaskRow
                      key={`${t.cup_id}:${t.action}:${idx}`}
                      refKey={`Arm1:${t.cup_id}:${t.action}`}
                      registerRef={(k, el) => { if (el) itemRefs.current[k] = el; }}
                      action={t.action}
                      cup={t.cup_id}
                      status={t.status}
                      taskTimings={taskTimings}
                      currentTime={currentTime}
                    />
                  ))}
                </div>
              )}
            </div>
          </div>

          {/* Right Column - Arm 2 Tasks (Live) */}
          <div className="flex flex-col h-full overflow-hidden">
            <h3 className="text-xs sm:text-sm font-semibold text-gray-900 mb-2 sm:mb-3 flex-shrink-0">Robot Arm 2</h3>
            <div ref={arm2ContainerRef} className="relative flex-1 overflow-y-auto pr-1 sm:pr-2 scroll-smooth pb-3 sm:pb-4">
              {schedulerTasks.Arm2.length === 0 ? (
                <div className="text-xs text-gray-400">No tasks yet.</div>
              ) : (
                <div className="space-y-2 sm:space-y-3">
                  {schedulerTasks.Arm2.map((t, idx) => (
                    <TaskRow
                      key={`${t.cup_id}:${t.action}:${idx}`}
                      refKey={`Arm2:${t.cup_id}:${t.action}`}
                      registerRef={(k, el) => { if (el) itemRefs.current[k] = el; }}
                      action={t.action}
                      cup={t.cup_id}
                      status={t.status}
                      taskTimings={taskTimings}
                      currentTime={currentTime}
                    />
                  ))}
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
} 

function TaskRow({ action, cup, status, refKey, registerRef, taskTimings, currentTime }) {
  const taskKey = `${cup}:${action}`;
  const timing = taskTimings[taskKey];
  
  // Calculate elapsed time
  const getElapsedTime = () => {
    if (!timing) return null;
    
    const isInProgress = status === 'in_progress' || status === 'submitted';
    const elapsed = isInProgress 
      ? (currentTime - timing.startTime) 
      : timing.elapsedTime;
    
    if (elapsed === null || elapsed === undefined) return null;
    
    const seconds = Math.floor(elapsed / 1000);
    const minutes = Math.floor(seconds / 60);
    const remainingSeconds = seconds % 60;
    
    if (minutes > 0) {
      return `${minutes}m ${remainingSeconds}s`;
    }
    return `${seconds}s`;
  };
  
  const elapsedTime = getElapsedTime();
  
  const getBadge = (s) => {
    if (s === 'completed') return <span className="text-[10px] sm:text-xs font-medium bg-green-100 text-green-800 rounded-full px-2 sm:px-3 py-0.5 sm:py-1 whitespace-nowrap">Completed</span>;
    if (s === 'failed') return <span className="text-[10px] sm:text-xs font-medium bg-white-100 text-red-800 rounded-full px-2 sm:px-3 py-0.5 sm:py-1 whitespace-nowrap">Failed</span>;
    if (s === 'in_progress' || s === 'submitted') return <span className="text-[10px] sm:text-xs font-medium bg-amber-100 text-amber-800 rounded-full px-2 sm:px-3 py-0.5 sm:py-1 whitespace-nowrap">In&nbsp;Progress</span>;
    if (s === 'cancelled') return <span className="text-[10px] sm:text-xs font-medium bg-gray-200 text-gray-600 rounded-full px-2 sm:px-3 py-0.5 sm:py-1 whitespace-nowrap">Cancelled</span>;
    return <span className="text-[10px] sm:text-xs font-medium bg-gray-100 text-gray-800 rounded-full px-2 sm:px-3 py-0.5 sm:py-1 whitespace-nowrap">Pending</span>;
  };

  const getIcon = (s) => {
    if (s === 'failed') return stop;
    if (s === 'completed') return lighting; // success style handled by bg
    if (s === 'in_progress' || s === 'submitted') return progressing;
    return circledots;
  };

  const containerClasses = (() => {
    if (status === 'failed') return 'border-2 border-red-400 bg-red-50';
    if (status === 'completed') return 'border border-green-200 bg-green-50';
    if (status === 'in_progress' || status === 'submitted') return 'border-2 border-amber-400 bg-amber-50';
    if (status === 'cancelled') return 'border border-gray-200 bg-gray-50 opacity-80';
    return 'border border-gray-200 bg-white';
  })();

  const iconBg = (() => {
    if (status === 'failed') return 'bg-red-600';
    if (status === 'completed') return 'bg-green-600';
    if (status === 'in_progress' || status === 'submitted') return 'bg-amber-500';
    if (status === 'cancelled') return 'bg-gray-400';
    return 'bg-gray-400';
  })();

  return (
    <div ref={(el) => registerRef && registerRef(refKey, el)} className={`relative flex items-start space-x-2 sm:space-x-4 p-2 sm:p-3 rounded-lg ${containerClasses}`}>
      <div className={`w-8 h-8 sm:w-10 sm:h-10 ${iconBg} rounded-full flex items-center justify-center flex-shrink-0`}>
        {status === 'in_progress' || status === 'submitted' ? (
          <svg className="animate-spin w-4 h-4 sm:w-5 sm:h-5 text-white" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
            <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
            <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
          </svg>
        ) : (
          <img src={getIcon(status)} alt="" className="w-4 h-4 sm:w-5 sm:h-5" />
        )}
      </div>
      <div className="flex-1 min-w-0">
        <div className="flex items-center justify-between gap-2 mb-1">
          <span className="text-xs sm:text-sm font-medium text-gray-600 flex-shrink-0">
            Cup {cup.split('-')[1] || cup}
          </span>
          {elapsedTime && (
            <span className="text-xs font-mono text-gray-500 bg-gray-100 px-1.5 sm:px-2 py-0.5 rounded flex-shrink-0">
              {elapsedTime}
            </span>
          )}
        </div>
        <div className="flex flex-col sm:flex-row sm:items-center gap-1 sm:gap-3">
          <span className="text-xs sm:text-sm font-medium text-gray-900 break-words line-clamp-2">{action}</span>
          <div className="flex-shrink-0">
            {getBadge(status)}
          </div>
        </div>
      </div>
    </div>
  );
}