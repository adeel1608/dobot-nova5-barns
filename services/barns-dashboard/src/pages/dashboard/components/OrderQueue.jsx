import React, { useState, useRef, useEffect } from 'react';
import { DndContext, closestCenter } from '@dnd-kit/core';
import Swal from 'sweetalert2';
import {
  arrayMove,
  SortableContext,
  useSortable,
  verticalListSortingStrategy
} from '@dnd-kit/sortable';
import { CSS } from '@dnd-kit/utilities';
import useStore from '../../../store';
import { useTranslation } from '../../../store/translationsStore';
import { SERVICE_OFFLINE_MESSAGE } from '../../../utils/errorHandler';
import { useIsPosMode } from '../../../store/displayStore';
import deleteIcon from '../../../assets/delete.png';
import OrderDetailModal from './OrderDetailModal';

/** Status dot — colored circle indicating order state, no text label. */
function StatusDot({ status }) {
  const colorMap = {
    PROCESSING: 'bg-orange-500',
    STOPPING: 'bg-amber-400',
    COMPLETED: 'bg-green-600',
    QUEUED: 'bg-blue-500',
    HALTED: 'bg-yellow-500',
    STOPPED: 'bg-red-500',
    ERROR: 'bg-red-500',
    CANCELLED: 'bg-red-500',
  };
  const color = colorMap[status?.toUpperCase()] || 'bg-gray-400';
  const isActive = status === 'PROCESSING';
  return (
    <span
      className={`inline-block w-2.5 h-2.5 rounded-full flex-shrink-0 ${color}${isActive ? ' animate-pulse' : ''}`}
      title={status}
    />
  );
}

function SortableItem({ order, index, onStartOrder, onStopOrder, onResumeOrder, onDeleteOrder, onViewDetails, onReorderOrder, isStarting, isStopping, isResuming, isDeleting, isReordering, getStatusBadge, t }) {

  const {
    attributes, listeners, setNodeRef,
    transform, transition, isDragging
  } = useSortable({ id: order.id });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.5 : 1,
  };

  const disableDrag = index === 0 && (order.status === 'PROCESSING' || order.status === 'STOPPING');

  // Determine if buttons should be disabled (only COMPLETED orders are fully disabled)
  const isDisabled = order.status === 'COMPLETED';

  // Get status badge with colored dot
  const getStatusBadgeWithDot = (status) => {
    switch (status) {
      case 'PROCESSING':
        return (
          <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-semibold bg-orange-100 text-orange-800">
            <div className="w-2.5 h-2.5 bg-orange-500 rounded-full mr-2"></div>
            {t('processing')}
          </span>
        );
      case 'STOPPING':
        return (
          <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-semibold bg-amber-100 text-amber-800">
            <svg className="animate-spin w-3.5 h-3.5 mr-2" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
              <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
              <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
            </svg>
            {t('stoppingLabel')}
          </span>
        );
      case 'COMPLETED':
        return (
          <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-semibold bg-green-100 text-green-800">
            <div className="w-2.5 h-2.5 bg-green-600 rounded-full mr-2"></div>
            {t('completed')}
          </span>
        );
      case 'QUEUED':
        return (
          <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-semibold bg-blue-100 text-blue-800">
            <div className="w-2.5 h-2.5 bg-blue-600 rounded-full mr-2"></div>
            {t('queued')}
          </span>
        );
      case 'CANCELLED':
      case 'ERROR':
      case 'STOPPED':
        return (
          <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-semibold bg-red-100 text-red-800">
            <div className="w-2.5 h-2.5 bg-red-600 rounded-full mr-2"></div>
            {status === 'CANCELLED' ? t('cancelled') : status === 'ERROR' ? t('error') : t('stopped')}
          </span>
        );
      case 'HALTED':
        return (
          <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-semibold bg-yellow-100 text-yellow-800">
            <div className="w-2.5 h-2.5 bg-yellow-500 rounded-full mr-2"></div>
            {t('halted')}
          </span>
        );
      default:
        return (
          <span className="inline-flex items-center px-3 py-1 rounded-full text-sm font-semibold bg-gray-100 text-gray-800">
            <div className="w-2.5 h-2.5 bg-gray-500 rounded-full mr-2"></div>
            {status}
          </span>
        );
    }
  };

  const isPosMode = useIsPosMode();

  // Visual highlight for processing/stopping/stopped orders
  const isProcessing = order.status === 'PROCESSING';
  const isInStoppingState = order.status === 'STOPPING';
  const isStopped = order.status === 'STOPPED';
  const containerClasses = isProcessing
    ? 'mb-2 p-3 rounded-lg border-2 border-orange-400 bg-orange-50 shadow-lg transition-all duration-200 ring-2 ring-orange-200'
    : isInStoppingState
      ? 'mb-2 p-3 rounded-lg border-2 border-amber-400 bg-amber-50 shadow-lg transition-all duration-200 ring-2 ring-amber-200'
      : isStopped
        ? 'mb-2 p-3 rounded-lg border-2 border-red-300 shadow-lg ring-2 ring-red-200 animate-blink-red'
        : 'mb-2 p-3 rounded-lg border border-gray-200 bg-white hover:shadow-sm transition-shadow duration-200';

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={`${containerClasses} cursor-pointer`}
      onClick={() => onViewDetails(order)}
    >
      {isPosMode ? (
        /* POS mode: row 1 = ID + status dot, row 2 = buttons only */
        <div className="flex flex-col gap-1.5">
          {/* Row 1: order ID on left, status dot on far right */}
          <div className="flex items-center justify-between">
            <span className="text-sm font-bold leading-tight">{t('orderId')}: {order.id}</span>
            <StatusDot status={order.status} />
          </div>
          {/* Row 2: action buttons, primary action fills available width */}
          <div className="flex items-center gap-1.5 w-full">
            <div className="flex items-center gap-1.5 w-full">
              {order.status === 'PROCESSING' ? (
                <button type="button" onClick={(e) => { e.stopPropagation(); onStopOrder(order.id); }} disabled={isStopping === order.id}
                  className={`flex-1 py-1 rounded text-xs font-bold transition-colors ${isStopping === order.id ? 'bg-red-300 text-white cursor-not-allowed' : 'bg-red-600 text-white hover:bg-red-700'}`}>
                  {isStopping === order.id ? t('stoppingLabel') : t('stop')}
                </button>
              ) : order.status === 'STOPPING' ? (
                <button type="button" disabled className="flex-1 py-1 rounded text-xs font-bold bg-amber-300 text-white cursor-not-allowed" onClick={(e) => e.stopPropagation()}>
                  {t('stoppingLabel')}
                </button>
              ) : order.status === 'STOPPED' ? (
                <button type="button" onClick={(e) => { e.stopPropagation(); onResumeOrder(order.id); }} disabled={isResuming === order.id}
                  className={`flex-1 py-1 rounded text-xs font-bold transition-colors ${isResuming === order.id ? 'bg-blue-300 text-white cursor-not-allowed' : 'text-white'}`}
                  style={{ backgroundColor: isResuming === order.id ? undefined : '#00754A' }}>
                  {isResuming === order.id ? t('resuming') : t('resume')}
                </button>
              ) : (
                <button type="button" onClick={(e) => { e.stopPropagation(); onStartOrder(order.id); }} disabled={isDisabled || isStarting === order.id}
                  className={`flex-1 py-1 rounded text-xs font-bold transition-colors ${isDisabled ? 'bg-gray-300 text-gray-500 cursor-not-allowed' : isStarting === order.id ? 'bg-blue-300 text-white cursor-not-allowed' : 'text-white'}`}
                  style={{ backgroundColor: isDisabled ? undefined : isStarting === order.id ? undefined : '#00754A' }}>
                  {isStarting === order.id ? t('starting') : t('start')}
                </button>
              )}
              <button type="button" onClick={(e) => { e.stopPropagation(); onReorderOrder(order); }} disabled={isReordering === order.id}
                className={`p-1 rounded flex items-center justify-center ${isReordering === order.id ? 'text-green-300 cursor-not-allowed' : 'text-green-600 hover:text-green-800'}`}
                style={{ border: '1.5px solid #22c55e', boxShadow: 'none', backgroundColor: 'transparent' }}
                aria-label={t('reorder')}>
                {isReordering === order.id
                  ? <svg className="animate-spin h-4 w-4" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24"><circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" /><path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" /></svg>
                  : <svg className="h-4 w-4" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth="2"><path strokeLinecap="round" strokeLinejoin="round" d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" /></svg>}
              </button>
              <button type="button" onClick={(e) => { e.stopPropagation(); onDeleteOrder && onDeleteOrder(order.id); }}
                disabled={isDeleting || order.status === 'STOPPING' || order.status === 'PROCESSING'}
                className="text-xs px-1.5 py-1 rounded flex items-center justify-center text-white hover:opacity-90 disabled:opacity-30 disabled:cursor-not-allowed disabled:grayscale shadow-none"
                style={{ height: '1.75rem', boxShadow: 'none' }} aria-label={t('delete')}>
                {isDeleting
                  ? <svg className="animate-spin h-3 w-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24"><circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4" /><path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z" /></svg>
                  : <img src={deleteIcon} alt="" aria-hidden="true" className="w-8 h-8" />}
              </button>
            </div>
          </div>
        </div>
      ) : (
        /* Default layout: ID + status on left, buttons on right (single row) */
        <div className="flex items-center justify-between">
          {/* Left side - ID and Status */}
          <div className="flex flex-col space-y-1.5">
            <span className="text-base font-bold">{t('orderId')}: {order.id}</span>
            {getStatusBadgeWithDot(order.status)}
          </div>

          {/* Right side - Action Buttons */}
          <div className="flex items-center space-x-2">
            {/* Dynamic Action Button (Start/Stop/Resume) */}
            {order.status === 'PROCESSING' ? (
              // Stop Button for Processing Orders
              <button
                type="button"
                onClick={(e) => { e.stopPropagation(); onStopOrder(order.id); }}
                disabled={isStopping === order.id}
                className={`px-4 py-2 rounded text-sm font-bold transition-colors ${isStopping === order.id
                    ? 'bg-red-300 text-white cursor-not-allowed'
                    : 'bg-red-600 text-white hover:bg-red-700'
                  }`}
                title={isStopping === order.id ? t('stoppingLabel') : t('stop')}
              >
                {isStopping === order.id ? t('stoppingLabel') : t('stop')}
              </button>
            ) : order.status === 'STOPPING' ? (
              // Stopping - button disabled while waiting
              <button
                type="button"
                disabled={true}
                className="px-4 py-2 rounded text-sm font-bold bg-amber-300 text-white cursor-not-allowed"
                title={t('stoppingLabel')}
                onClick={(e) => e.stopPropagation()}
              >
                <span className="inline-flex items-center">
                  {t('stoppingLabel')}
                </span>
              </button>
            ) : order.status === 'STOPPED' ? (
              // Resume Button for Stopped Orders
              <button
                type="button"
                onClick={(e) => { e.stopPropagation(); onResumeOrder(order.id); }}
                disabled={isResuming === order.id}
                className={`px-4 py-2 rounded text-sm font-bold transition-colors ${isResuming === order.id
                    ? 'bg-blue-300 text-white cursor-not-allowed'
                    : 'text-white'
                  }`}
                style={{
                  backgroundColor: isResuming === order.id ? undefined : '#00754A'
                }}
                title={isResuming === order.id ? t('resuming') : t('resume')}
              >
                {isResuming === order.id ? t('resuming') : t('resume')}
              </button>
            ) : (
              // Start Button for Queued/Cancelled/Error Orders
              <button
                type="button"
                onClick={(e) => { e.stopPropagation(); onStartOrder(order.id); }}
                disabled={isDisabled || isStarting === order.id}
                className={`px-4 py-2 rounded text-sm font-bold transition-colors ${isDisabled
                    ? 'bg-gray-300 text-gray-500 cursor-not-allowed'
                    : isStarting === order.id
                      ? 'bg-blue-300 text-white cursor-not-allowed'
                      : 'text-white'
                  }`}
                style={{
                  backgroundColor: isDisabled
                    ? undefined
                    : isStarting === order.id
                      ? undefined
                      : '#00754A'
                }}
                title={isDisabled ? '' : t('start')}
              >
                {isStarting === order.id ? t('starting') : t('start')}
              </button>
            )}

            {/* Reorder Button - icon only */}
            <button
              type="button"
              onClick={(e) => { e.stopPropagation(); onReorderOrder(order); }}
              disabled={isReordering === order.id}
              className={`p-1 rounded transition-colors flex items-center justify-center ${isReordering === order.id
                  ? 'text-green-300 cursor-not-allowed'
                  : 'text-green-600 hover:text-green-800'
                }`}
              style={{ border: '1.5px solid #22c55e', boxShadow: 'none', backgroundColor: 'transparent' }}
              title={isReordering === order.id ? t('reordering') : t('reorder')}
              aria-label={t('reorder')}
            >
              {isReordering === order.id ? (
                <svg className="animate-spin h-5 w-5" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                </svg>
              ) : (
                <svg className="h-5 w-5" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" stroke="currentColor" strokeWidth="2">
                  <path strokeLinecap="round" strokeLinejoin="round" d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
                </svg>
              )}
            </button>

            {/* Delete Button - Icon only */}
            <button
              type="button"
              onClick={(e) => { e.stopPropagation(); onDeleteOrder && onDeleteOrder(order.id); }}
              disabled={isDeleting || order.status === 'STOPPING' || order.status === 'PROCESSING'}
              className={`text-xs px-2 py-1 rounded flex items-center justify-center text-white hover:opacity-90 disabled:opacity-30 disabled:cursor-not-allowed disabled:grayscale shadow-none transition-all`}
              title={
                isDeleting ? t('deleting')
                  : order.status === 'STOPPING' ? t('stoppingLabel')
                    : order.status === 'PROCESSING' ? t('processing')
                      : t('delete')
              }
              aria-label={
                isDeleting ? t('deleting')
                  : order.status === 'STOPPING' ? t('stoppingLabel')
                    : order.status === 'PROCESSING' ? t('processing')
                      : t('delete')
              }
              style={{ height: '2rem', boxShadow: 'none' }}
            >
              {isDeleting ? (
                <svg className="animate-spin h-3 w-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                </svg>
              ) : (
                <img src={deleteIcon} alt="" aria-hidden="true" className="w-10 h-10" />
              )}
            </button>
          </div>
        </div>
      )} {/* end default mode */}
    </div>
  );
}

function OrderQueue({ connectionStatus }) {
  const { t } = useTranslation('dashboard');
  const {
    orders,
    ordersTotal,
    ordersHasMore,
    fetchOrders,
    sendReorder,
    createOrder,
    startOrder,
    stopOrder,
    resumeOrder,
    deleteOrder,
    loadMoreOrders,
    isLoading,
    errors,
    clearError,
    navigate
  } = useStore();

  const [selectedOrder, setSelectedOrder] = useState(null);
  const [filterStatus, setFilterStatus] = useState('ALL');
  const [searchTerm, setSearchTerm] = useState('');
  const [startingOrderId, setStartingOrderId] = useState(null);
  const [stoppingOrderId, setStoppingOrderId] = useState(null);
  const [resumingOrderId, setResumingOrderId] = useState(null);
  const [deletingOrderId, setDeletingOrderId] = useState(null);
  const [reorderingOrderId, setReorderingOrderId] = useState(null);
  const logsEndRef = useRef(null);
  const scrollContainerRef = useRef(null);

  // Auto-refresh when there's a STOPPING order to ensure UI updates quickly
  useEffect(() => {
    const stoppingOrders = orders.filter(o => o.status === 'STOPPING' || o.status === 'PROCESSING');

    if (stoppingOrders.length > 0) {
      console.log('📡 Active STOPPING/PROCESSING orders detected, enabling fast polling');
      // Poll every 1 second while there are active stopping/processing orders
      const interval = setInterval(() => {
        console.log('🔄 Polling for order updates (STOPPING/PROCESSING active)');
        fetchOrders();
      }, 1000); // Changed from 2000ms to 1000ms for faster updates

      return () => {
        console.log('📡 Stopping fast polling');
        clearInterval(interval);
      };
    }
  }, [orders, fetchOrders]);

  const displayOrders = orders;

  const getStatusBadge = (status) => {
    switch (status) {
      case 'PROCESSING':
        return <span className="bg-yellow-100 text-yellow-800 text-xs font-medium px-2.5 py-0.5 rounded">{t('processing')}</span>;
      case 'STOPPING':
        return <span className="bg-amber-100 text-amber-800 text-xs font-medium px-2.5 py-0.5 rounded">{t('stoppingLabel')}</span>;
      case 'COMPLETED':
        return <span className="bg-green-100 text-green-800 text-xs font-medium px-2.5 py-0.5 rounded">{t('completed')}</span>;
      case 'HALTED':
        return <span className="bg-orange-100 text-orange-800 text-xs font-medium px-2.5 py-0.5 rounded">{t('halted')}</span>;
      case 'STOPPED':
        return <span className="bg-red-100 text-red-800 text-xs font-medium px-2.5 py-0.5 rounded">{t('stopped')}</span>;
      case 'ERROR':
        return <span className="bg-red-100 text-red-800 text-xs font-medium px-2.5 py-0.5 rounded">{t('error')}</span>;
      case 'CANCELLED':
        return <span className="bg-gray-100 text-gray-800 text-xs font-medium px-2.5 py-0.5 rounded">{t('cancelled')}</span>;
      default:
        return <span className="bg-blue-100 text-blue-800 text-xs font-medium px-2.5 py-0.5 rounded">{t('queued')}</span>;
    }
  };

  const handleDragEnd = ({ active, over }) => {
    if (!over || errors.orders) return; // Prevent reordering if there's an API error

    const oldIndex = displayOrders.findIndex(o => o.id === active.id);
    const newIndex = displayOrders.findIndex(o => o.id === over.id);
    if (oldIndex === 0 && (displayOrders[0].status === 'PROCESSING' || displayOrders[0].status === 'STOPPING')) return; // block moving processing/stopping

    const newOrders = arrayMove(displayOrders, oldIndex, newIndex);
    sendReorder(newOrders);
  };

  const handleStartOrder = async (orderId) => {
    console.log('🚀 handleStartOrder called with orderId:', orderId);
    console.log('🚀 Current startingOrderId:', startingOrderId);

    setStartingOrderId(orderId);
    try {
      console.log('🚀 About to call startOrder from store...');
      const success = await startOrder(orderId);
      console.log('🚀 startOrder returned:', success);

      if (success) {
        console.log(`✅ Order ${orderId} started successfully`);
        // Show success feedback
        setTimeout(() => {
          setStartingOrderId(null);
        }, 1000); // Keep loading state for 1 second to show feedback
      } else {
        console.error(`❌ Failed to start order ${orderId}`);
        setStartingOrderId(null);
      }
    } catch (error) {
      console.error('🚀 Error in handleStartOrder:', error);
      setStartingOrderId(null);
    }
  };

  const handleStopOrder = async (orderId) => {
    console.log('🛑 handleStopOrder called with orderId:', orderId);

    setStoppingOrderId(orderId);
    try {
      console.log('🛑 About to call stopOrder from store...');
      const success = await stopOrder(orderId);
      console.log('🛑 stopOrder returned:', success);

      if (success) {
        console.log(`✅ Order ${orderId} stop command sent successfully`);
        // Clear the stopping state immediately - the order status will be managed by the store
        setStoppingOrderId(null);
      } else {
        console.error(`❌ Failed to stop order ${orderId}`);
        setStoppingOrderId(null);
      }
    } catch (error) {
      console.error('🛑 Error in handleStopOrder:', error);
      setStoppingOrderId(null);
    }
  };

  const handleResumeOrder = async (orderId) => {
    console.log('🔄 handleResumeOrder called with orderId:', orderId);

    setResumingOrderId(orderId);
    try {
      console.log('🔄 About to call resumeOrder from store...');
      const success = await resumeOrder(orderId);
      console.log('🔄 resumeOrder returned:', success);

      if (success) {
        console.log(`✅ Order ${orderId} resumed successfully`);
        setTimeout(() => {
          setResumingOrderId(null);
        }, 1000);
      } else {
        console.error(`❌ Failed to resume order ${orderId}`);
        setResumingOrderId(null);
      }
    } catch (error) {
      console.error('🔄 Error in handleResumeOrder:', error);
      setResumingOrderId(null);
    }
  };

  const handleCalibrateOrder = async () => {
    try {
      // Create order using direct cups format since Calibrate is a manual item
      const calibrateOrder = {
        cups: [{
          type: 'Calibrate',
          size: '9oz',
          addons: [],
          ingredients: []
        }]
      };

      const success = await createOrder(calibrateOrder);

      if (success) {
        Swal.fire({
          icon: 'success',
          title: t('calibrate') + '!',
          text: t('calibrateOrderCreated'),
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      } else {
        Swal.fire({
          icon: 'error',
          title: t('failed'),
          text: t('calibrateOrderFailed'),
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      }
    } catch (error) {
      console.error('Error creating calibrate order:', error);
      Swal.fire({
        icon: 'error',
        title: t('error'),
        text: t('calibrateOrderFailed'),
        timer: 2500,
        timerProgressBar: true,
        showConfirmButton: false,
      });
    }
  };

  const handleReorderOrder = async (order) => {
    try {
      const cups = (order.cups || []).map(cup => ({
        type: cup.type || cup.drink_type || '',
        size: cup.size || cup.cup_size || 'regular',
        addons: Array.isArray(cup.addons) ? cup.addons : [],
        ingredients: cup.ingredients || []
      })).filter(c => c.type && c.type.trim() !== '');

      if (cups.length === 0) {
        Swal.fire({
          icon: 'warning',
          title: t('cannotReorder'),
          text: t('noValidCups'),
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
        return;
      }

      setReorderingOrderId(order.id);
      const success = await createOrder({ cups });

      if (success) {
        Swal.fire({
          icon: 'success',
          title: t('reordered'),
          text: t('reordered'),
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      } else {
        Swal.fire({
          icon: 'error',
          title: t('reorderFailed'),
          text: t('reorderFailed'),
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      }
    } catch (e) {
      console.error('Error in handleReorderOrder:', e);
      Swal.fire({
        icon: 'error',
        title: t('error'),
        text: t('unexpectedErrorReorder'),
        timer: 2500,
        timerProgressBar: true,
        showConfirmButton: false,
      });
    } finally {
      setReorderingOrderId(null);
    }
  };

  const handleDeleteOrder = async (orderId) => {
    console.log('🗑️ handleDeleteOrder called with orderId:', orderId);

    const order = displayOrders.find(o => o.id === orderId);
    const orderStatus = order?.status?.toUpperCase() || 'UNKNOWN';

    let swalConfig = {
      title: `${t('deleteOrderConfirm')} #${orderId}?`,
      icon: 'warning',
      showCancelButton: true,
      confirmButtonText: t('yesDelete'),
      cancelButtonText: t('cancel'),
      reverseButtons: true,
    };

    if (orderStatus === 'PROCESSING') {
      swalConfig = {
        ...swalConfig,
        title: `#${orderId} ${t('deleteOrderProcessing')}`,
        text: t('deleteOrderProcessing'),
      };
    } else {
      swalConfig = {
        ...swalConfig,
        text: t('cannotUndo'),
      };
    }

    const result = await Swal.fire(swalConfig);

    if (!result.isConfirmed) return;

    setDeletingOrderId(orderId);

    try {
      console.log('🗑️ About to call deleteOrder from store...');
      const success = await deleteOrder(orderId);
      console.log('🗑️ deleteOrder returned:', success);

      if (success) {
        Swal.fire({
          icon: 'success',
          title: t('deleted'),
          text: t('deletedOrder'),
          timer: 3000,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      } else {
        Swal.fire({
          icon: 'error',
          title: t('failed'),
          text: t('couldNotDelete'),
          timer: 3000,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      }
    } catch (error) {
      console.error('🗑️ Error in handleDeleteOrder:', error);
      Swal.fire({
        icon: 'error',
        title: t('error'),
        text: t('unexpectedErrorDeleting'),
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false,
      });
    } finally {
      setDeletingOrderId(null);
    }
  };

  const viewOrderDetails = (order) => {
    setSelectedOrder(order);
  };

  const closeOrderDetails = () => {
    setSelectedOrder(null);
  };

  const retryFetchOrders = () => {
    clearError('orders');
    fetchOrders();
  };

  const handleLoadMore = async () => {
    // Save current scroll position before loading
    const scrollContainer = scrollContainerRef.current;
    if (!scrollContainer) {
      await loadMoreOrders();
      return;
    }

    const scrollTop = scrollContainer.scrollTop;

    // Load more orders
    await loadMoreOrders();

    // Restore scroll position after React re-renders
    // Use setTimeout to ensure DOM has fully updated
    setTimeout(() => {
      if (scrollContainer) {
        // Maintain the same scroll position so user stays looking at the same content
        scrollContainer.scrollTop = scrollTop;
      }
    }, 0);
  };

  const formatOrderForDisplay = (order) => {
    return {
      ...order,
      // Map cup data to display format
      itemName: order.cups && order.cups.length > 0
        ? order.cups.map(cup => `${cup.drink_type || cup.type} (${cup.cup_size || cup.size})`).join(', ')
        : 'Unknown Item',
      // Map status
      status: order.status?.toUpperCase() || 'QUEUED',
      // Check if manual intervention is required
      manualRequired: order.cups && order.cups.some(cup =>
        cup.addons && cup.addons.includes('manual_required')
      ),
      // Format timestamps
      createdAt: order.created_at ? new Date(order.created_at).toLocaleString() : 'N/A',
      startedAt: order.started_at ? new Date(order.started_at).toLocaleString() : 'N/A',
      completedAt: order.completed_at ? new Date(order.completed_at).toLocaleString() : 'N/A',
      // For "Updated" field, show the most recent timestamp
      updatedAt: order.completed_at ? new Date(order.completed_at).toLocaleString() :
        order.started_at ? new Date(order.started_at).toLocaleString() : 'N/A'
    };
  };

  // Filter orders based on status and search term
  const filteredOrders = displayOrders.map(formatOrderForDisplay).filter(order => {
    // Filter by status
    if (filterStatus !== 'ALL' && order.status !== filterStatus) {
      return false;
    }

    // Filter by search term
    if (searchTerm && !order.itemName.toLowerCase().includes(searchTerm.toLowerCase())) {
      return false;
    }

    return true;
  });

  return (
    <div className="bg-white rounded-lg shadow-xl  flex flex-col h-full">
      {/* Header */}
      <div className="flex flex-col space-y-3 p-3 md:p-4 border-b border-gray-200 flex-shrink-0">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <button
              onClick={() => {
                navigate?.('newOrder');
                window.location.hash = '#/newOrder';
              }}
              className="text-sm rounded font-bold transition-colors duration-300 px-4 py-2 cursor-pointer flex items-center gap-1.5 text-white hover:opacity-90"
              style={{ backgroundColor: '#059669', color: 'white' }}
            >
              <svg className="w-4 h-4 flex-shrink-0" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="3">
                <path strokeLinecap="round" strokeLinejoin="round" d="M12 4v16m8-8H4" />
              </svg>
              {t('newOrder')}
            </button>
            {errors.orders && (
              <span className="inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-red-100 text-red-800">
                {t('apiError')}
              </span>
            )}
          </div>

          <div className="flex items-center gap-2">
            <div
              className={`w-2 h-2 rounded-full ${connectionStatus ? 'bg-green-500' : 'bg-red-500'
                }`}
            ></div>
            <h2
              onClick={handleCalibrateOrder}
              className="text-sm rounded font-small transition-colors duration-300 px-4 py-2 cursor-pointer barns-dark-bg text-white hover:barns-bg"
              style={{ color: 'white' }}
            >
              {t('calibrate')}
            </h2>
          </div>
        </div>

        <div className="flex flex-col sm:flex-row space-y-2 sm:space-y-0 sm:space-x-3">
          <input
            type="text"
            placeholder={t('searchOrders')}
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.target.value)}
            className="flex-1 px-3 py-2 barns-border-dark   focus:ring-0  text-sm"
          />
          <select
            value={filterStatus}
            onChange={(e) => setFilterStatus(e.target.value)}
            className="px-4 py-2 barns-border-dark focus:ring-2  text-sm sm:w-auto"
          >
            <option value="ALL">{t('allOrders')}</option>
            <option value="QUEUED">{t('queued')}</option>
            <option value="PROCESSING">{t('processing')}</option>
            <option value="STOPPING">{t('stopping')}</option>
            <option value="HALTED">{t('halted')}</option>
            <option value="COMPLETED">{t('completed')}</option>
            <option value="STOPPED">{t('stopped')}</option>
            <option value="ERROR">{t('error')}</option>
            <option value="CANCELLED">{t('cancelled')}</option>
          </select>

        </div>
      </div>

      {/* API Error display */}
      {errors.orders && (
        <div className="border-b border-red-200 bg-red-50 px-4 py-2 text-sm text-red-700 flex justify-between items-center flex-shrink-0">
          <div>
            <span className="font-medium">{t('apiError')}:</span> {errors.orders === SERVICE_OFFLINE_MESSAGE ? t('serviceOfflineUnreachable') : errors.orders}
            {errors.orders !== SERVICE_OFFLINE_MESSAGE && (
              <p className="text-xs mt-1">{t('pleaseCheckOms')}</p>
            )}
          </div>
          <button
            onClick={retryFetchOrders}
            className="px-2 py-1 bg-red-100 hover:bg-red-200 text-red-800 rounded text-xs font-medium"
          >
            {t('refresh')}
          </button>
        </div>
      )}

      {/* Scrollable content area */}
      <div className="flex-1 overflow-hidden">
        <div ref={scrollContainerRef} className="h-full overflow-y-auto p-4">
          {/* Orders List */}
          {isLoading ? (
            <div className="flex justify-center items-center p-8">
              <svg className="animate-spin h-8 w-8 text-blue-500" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
              </svg>
            </div>
          ) : filteredOrders.length === 0 ? (
            <div className="text-center p-8 text-gray-500">
              {searchTerm || filterStatus !== 'ALL' ? (
                <p>{t('noOrdersMatch')}</p>
              ) : errors.orders ? (
                <p>{t('unableToLoadOrders')}</p>
              ) : (
                <p>{t('noOrdersInQueue')}</p>
              )}
            </div>
          ) : (
            <DndContext collisionDetection={closestCenter} onDragEnd={handleDragEnd}>
              <SortableContext items={filteredOrders.map(o => o.id)} strategy={verticalListSortingStrategy}>
                <div className="space-y-2">
                  {filteredOrders.map((order, idx) => (
                    <SortableItem
                      key={order.id}
                      order={order}
                      index={idx}
                      onStartOrder={handleStartOrder}
                      onStopOrder={handleStopOrder}
                      onResumeOrder={handleResumeOrder}
                      onDeleteOrder={handleDeleteOrder}
                      onViewDetails={viewOrderDetails}
                      onReorderOrder={handleReorderOrder}
                      isStarting={startingOrderId === order.id}
                      isStopping={stoppingOrderId === order.id}
                      isResuming={resumingOrderId === order.id}
                      isDeleting={deletingOrderId === order.id}
                      isReordering={reorderingOrderId === order.id}
                      getStatusBadge={getStatusBadge}
                      t={t}
                    />
                  ))}
                </div>
              </SortableContext>
            </DndContext>
          )}

          {/* Show More Button */}
          {filteredOrders.length > 0 && ordersHasMore && (
            <div className="mt-4 flex justify-center">
              <button
                onClick={handleLoadMore}
                disabled={isLoading}
                className="px-6 py-3 bg-gray-100 hover:bg-gray-200 text-gray-700 rounded-lg text-sm font-medium transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed flex items-center space-x-2"
              >
                {isLoading ? (
                  <>
                    <svg className="animate-spin h-4 w-4" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                      <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                      <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                    </svg>
                    <span>{t('loading')}</span>
                  </>
                ) : (
                  <>
                    <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
                    </svg>
                    <span>{t('showMore')} ({ordersTotal - orders.length} {t('remaining')})</span>
                  </>
                )}
              </button>
            </div>
          )}
        </div>
      </div>

      {/* Order Details Modal */}
      {selectedOrder && (
        <OrderDetailModal
          order={selectedOrder}
          onClose={closeOrderDetails}
          onStartOrder={handleStartOrder}
          onResumeOrder={handleResumeOrder}
          getStatusBadge={getStatusBadge}
          t={t}
        />
      )}
    </div>
  );
}

export default OrderQueue;
