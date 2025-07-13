/** @type {import('tailwindcss').Config} */
export default {
    content: [
        "./index.html",
        "./src/**/*.{js,ts,jsx,tsx}",
    ],
    theme: {
        extend: {
            colors: {
                // BARNS Coffee Theme Colors (extending default colors, not replacing)
                'coffee': {
                    'dark': '#2D1B14',
                    'medium': '#4A2C1A', 
                    'light': '#8B4513',
                },
                'cream': '#F5F1EB',
                'amber': {
                    '300': '#F3D5A7',
                    '400': '#E6C885',
                    '500': '#D4A574',
                    '600': '#CD853F',
                },
                'coffee-green': '#6B8E3D',
                'caramel': '#CD853F',
                'steam-white': '#FEFEFE',
                'shadow-gray': '#4A4A4A',
            },
            fontFamily: {
                'sans': ['Titillium Web', 'sans-serif'],
            },
            boxShadow: {
                'coffee': '0 4px 6px rgba(45, 27, 20, 0.07), 0 1px 3px rgba(45, 27, 20, 0.06)',
                'coffee-lg': '0 8px 15px rgba(45, 27, 20, 0.12), 0 3px 6px rgba(45, 27, 20, 0.08)',
                'coffee-xl': '0 4px 6px rgba(45, 27, 20, 0.3)',
            },
            animation: {
                'coffee-pulse': 'coffee-pulse 2s ease-in-out infinite',
                'steam': 'steam 3s ease-in-out infinite',
            },
            keyframes: {
                'coffee-pulse': {
                    '0%, 100%': {
                        opacity: '1',
                        transform: 'scale(1)',
                    },
                    '50%': {
                        opacity: '0.8',
                        transform: 'scale(1.02)',
                    },
                },
                'steam': {
                    '0%': {
                        transform: 'translateY(0) scaleX(1)',
                        opacity: '0.7',
                    },
                    '50%': {
                        transform: 'translateY(-10px) scaleX(1.1)',
                        opacity: '0.4',
                    },
                    '100%': {
                        transform: 'translateY(-20px) scaleX(0.9)',
                        opacity: '0',
                    },
                },
            },
            gradients: {
                'coffee-header': 'linear-gradient(135deg, #2D1B14 0%, #4A2C1A 50%, #8B4513 100%)',
                'coffee-bg': 'linear-gradient(135deg, #F5F1EB 0%, #F0EBE5 100%)',
            },
        },
    },
    plugins: [],
}
  