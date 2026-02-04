import React from 'react';
import useStore from '../../../store';
import { useTranslation } from '../../../store/translationsStore';

export default function OrderStatusPanel() {
  const { t } = useTranslation('dashboard');
  const { orderStats } = useStore();

  return (
    <div className="rounded-lg p-4 space-y-4">
      <div className="flex flex-col sm:flex-row gap-3">
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats?.total || 0}</div>
          <div className="text-sm font-semibold text-gray-600">{t('totalOrders')}</div>
        </div>
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats?.processing || 0}</div>
          <div className="text-sm font-semibold text-gray-600">{t('processing')}</div>
        </div>
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats?.queued || 0}</div>
          <div className="text-sm font-semibold text-gray-600">{t('inQueue')}</div>
        </div>
        <div className="bg-[#E6F1ED] p-4 rounded-lg flex flex-col justify-center items-center flex-1">
          <div className="text-3xl font-bold text-gray-700">{orderStats?.completed || 0}</div>
          <div className="text-sm font-semibold text-gray-600">{t('completed')}</div>
        </div>
      </div>
    </div>
  );
}
