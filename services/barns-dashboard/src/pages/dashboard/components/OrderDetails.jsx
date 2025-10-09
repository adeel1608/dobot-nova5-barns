import React, { useState, useEffect, useRef } from 'react';
import useStore from '../../../store';
import stop from '../../../assets/stop.png';
import coffee from '../../../assets/Coffee.png';
import lighting from '../../../assets/lighting.png';
import dots from '../../../assets/dots.png';
import progressing from '../../../assets/progressing.png';
import circledots from '../../../assets/circledots.png';
export default function OrderDetails() {
  const { orders, schedulerTasks, schedulerTaskStatus, schedulerStatusMessage } = useStore(state => ({
    orders: state.orders,
    schedulerTasks: state.schedulerTasks || { Arm1: [], Arm2: [] },
    schedulerTaskStatus: state.schedulerTaskStatus || {},
    schedulerStatusMessage: state.schedulerStatusMessage || null
  }));
  const [showTaskInterface, setShowTaskInterface] = useState(true); // State to control showing task interface
  const arm1ContainerRef = useRef(null);
  const arm2ContainerRef = useRef(null);
  const itemRefs = useRef({});

  // Find the currently processing order
  const processingOrder = orders?.find(order => 
    order.status?.toUpperCase() === 'PROCESSING'
  );

  // Show task interface if enabled, otherwise show idle message
  if (!showTaskInterface && !processingOrder) {
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

  const displayOrder = processingOrder ? formatOrderForDisplay(processingOrder) : null;

  // Auto-scroll to keep focus on in-progress task; choose the arm with least completed tasks
  useEffect(() => {
    try {
      const arm1 = schedulerTasks.Arm1 || [];
      const arm2 = schedulerTasks.Arm2 || [];

      const completed1 = arm1.filter(t => t.status === 'completed').length;
      const completed2 = arm2.filter(t => t.status === 'completed').length;

      // Pick arm with least completed tasks; if equal, prefer one with an in-progress task
      const armChoice = (() => {
        const hasIP1 = arm1.some(t => t.status === 'in_progress' || t.status === 'submitted');
        const hasIP2 = arm2.some(t => t.status === 'in_progress' || t.status === 'submitted');
        if (completed1 < completed2) return 'Arm1';
        if (completed2 < completed1) return 'Arm2';
        if (hasIP1 && !hasIP2) return 'Arm1';
        if (hasIP2 && !hasIP1) return 'Arm2';
        return 'Arm1';
      })();

      const container = armChoice === 'Arm1' ? arm1ContainerRef.current : arm2ContainerRef.current;
      const list = armChoice === 'Arm1' ? arm1 : arm2;
      if (!container || !list || list.length === 0) return;

      const inProgress = list.find(t => t.status === 'in_progress' || t.status === 'submitted');
      const pendingFirst = list.find(t => t.status === 'pending');
      const target = inProgress || pendingFirst || list.find(t => t.status === 'failed') || list[list.length - 1];
      if (!target) return;
      const key = `${armChoice}:${target.cup_id}:${target.action}`;
      const el = itemRefs.current[key];
      if (!el) return;

      const top = el.offsetTop - (container.clientHeight / 2) + (el.clientHeight / 2);
      container.scrollTo({ top: Math.max(0, top), behavior: 'smooth' });
    } catch {}
  }, [schedulerTasks]);

  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-200 flex flex-col h-full">
      {/* Header */}
      <div className="p-3 border-b border-gray-200 flex-shrink-0">
        <div className="flex items-center justify-between">
          <h2 className="text-lg font-semibold text-gray-900">Current Order</h2>
        </div>
      </div>

      {/* Task Management Interface */}
      <div className="flex-1 p-4">
        {schedulerStatusMessage && (
          <div className="mb-3 text-xs text-gray-500">{schedulerStatusMessage}</div>
        )}

        {/* Task Columns */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 h-full">
          {/* Left Column - Arm 1 Tasks (Live) */}
          <div className="flex flex-col h-full">
            <h3 className="text-sm font-semibold text-gray-900 mb-3 flex-shrink-0">Robot Arm 1</h3>
            <div ref={arm1ContainerRef} className="relative flex-1 overflow-y-auto pr-2 scroll-smooth">
              {schedulerTasks.Arm1.length === 0 ? (
                <div className="text-xs text-gray-400">No tasks yet.</div>
              ) : (
                <div className="space-y-4">
                  {schedulerTasks.Arm1.map((t, idx) => (
                    <TaskRow
                      key={`${t.cup_id}:${t.action}:${idx}`}
                      refKey={`Arm1:${t.cup_id}:${t.action}`}
                      registerRef={(k, el) => { if (el) itemRefs.current[k] = el; }}
                      action={t.action}
                      cup={t.cup_id}
                      status={t.status}
                    />
                  ))}
                </div>
              )}
            </div>
          </div>

          {/* Right Column - Arm 2 Tasks (Live) */}
          <div className="flex flex-col h-full">
            <h3 className="text-sm font-semibold text-gray-900 mb-3 flex-shrink-0">Robot Arm 2</h3>
            <div ref={arm2ContainerRef} className="relative flex-1 overflow-y-auto pr-2 scroll-smooth">
              {schedulerTasks.Arm2.length === 0 ? (
                <div className="text-xs text-gray-400">No tasks yet.</div>
              ) : (
                <div className="space-y-4">
                  {schedulerTasks.Arm2.map((t, idx) => (
                    <TaskRow
                      key={`${t.cup_id}:${t.action}:${idx}`}
                      refKey={`Arm2:${t.cup_id}:${t.action}`}
                      registerRef={(k, el) => { if (el) itemRefs.current[k] = el; }}
                      action={t.action}
                      cup={t.cup_id}
                      status={t.status}
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

function TaskRow({ action, cup, status, refKey, registerRef }) {
  const getBadge = (s) => {
    if (s === 'completed') return <span className="text-xs font-medium bg-green-100 text-green-800 rounded-full px-3 py-1">Completed</span>;
    if (s === 'failed') return <span className="text-xs font-medium bg-white-100 text-red-800 rounded-full px-3 py-1">Failed</span>;
    if (s === 'in_progress' || s === 'submitted') return <span className="text-xs font-medium bg-amber-100 text-amber-800 rounded-full px-3 py-1">In&nbsp;Progress</span>;
    if (s === 'cancelled') return <span className="text-xs font-medium bg-gray-200 text-gray-600 rounded-full px-3 py-1">Cancelled</span>;
    return <span className="text-xs font-medium bg-gray-100 text-gray-800 rounded-full px-3 py-1">Pending</span>;
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
    <div ref={(el) => registerRef && registerRef(refKey, el)} className={`relative flex items-start space-x-4 p-3 rounded-lg ${containerClasses}`}>
      <div className={`w-10 h-10 ${iconBg} rounded-full flex items-center justify-center flex-shrink-0`}>
        <img src={getIcon(status)} alt="" className="w-5 h-5" />
      </div>
      <div className="flex-1">
        <div className="flex items-center space-x-3">
          <span className="text-xs font-medium text-gray-600">{cup}</span>
        </div>
        <div className="flex items-center space-x-3">
          <span className="text-sm font-medium text-gray-900">{action}</span>
          {getBadge(status)}
        </div>
      </div>
    </div>
  );
}