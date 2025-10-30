-- File: ./services/validation/schema.sql

-- Connect to Validation database and set user context
\c barns_validation validation_user;

-- Set configuration for the session
SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;
SET default_tablespace = '';
SET default_table_access_method = heap;

--
-- Name: inventory; Type: TABLE; Schema: public; Owner: validation_user
--

CREATE TABLE IF NOT EXISTS public.inventory (
    id integer NOT NULL,
    ingredient_type character varying(50) NOT NULL,
    subtype character varying(50) NOT NULL,
    current_amount numeric(10,2) NOT NULL,
    last_updated timestamp without time zone DEFAULT CURRENT_TIMESTAMP
);

--
-- Name: inventory_id_seq; Type: SEQUENCE; Schema: public; Owner: validation_user
--

CREATE SEQUENCE IF NOT EXISTS public.inventory_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;

--
-- Name: inventory_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: validation_user
--

ALTER SEQUENCE public.inventory_id_seq OWNED BY public.inventory.id;

--
-- Name: inventory id; Type: DEFAULT; Schema: public; Owner: validation_user
--

ALTER TABLE ONLY public.inventory ALTER COLUMN id SET DEFAULT nextval('public.inventory_id_seq'::regclass);

--
-- Data for Name: inventory; Type: TABLE DATA; Schema: public; Owner: validation_user
--

INSERT INTO public.inventory (id, ingredient_type, subtype, current_amount, last_updated) VALUES
(1, 'coffee_beans', 'regular', 5000.00, '2025-07-13 10:43:31.892028'),
(2, 'coffee_beans', 'decaf', 3000.00, '2025-07-08 12:43:55.681776'),
(3, 'cups', 'cup_H7', 25.00, '2025-07-08 12:43:55.806483'),
(4, 'cups', 'cup_H9', 25.00, '2025-07-08 12:43:55.897223'),
(5, 'cups', 'cup_H12', 25.00, '2025-07-08 12:43:55.989214'),
(6, 'cups', 'cup_C7', 50.00, '2025-07-08 12:43:56.090518'),
(7, 'cups', 'cup_C9', 50.00, '2025-07-08 12:43:56.181739'),
(8, 'cups', 'cup_C12', 50.00, '2025-07-08 12:43:56.376387'),
(9, 'cups', 'cup_C16', 50.00, '2025-07-08 12:43:56.510802'),
(10, 'milk', 'whole_fat_milk', 1000.00, '2025-07-08 12:43:56.596484'),
(11, 'milk', 'low_fat_milk', 1000.00, '2025-07-08 12:43:56.674165'),
(12, 'milk', 'lactose_free_milk', 1000.00, '2025-07-08 12:43:56.756757'),
(13, 'milk', 'almond_milk', 1000.00, '2025-07-08 12:43:56.856309'),
(14, 'syrups', 'vanilla_syrup', 1000.00, '2025-07-08 12:47:59.745037'),
(15, 'syrups', 'caramel_syrup', 1000.00, '2025-07-08 12:43:57.065299'),
(16, 'syrups', 'hazelnut_syrup', 1000.00, '2025-07-08 12:43:57.163559'),
(17, 'syrups', 'peached_iced_syrup', 1000.00, '2025-07-08 12:43:57.259803'),
(18, 'syrups', 'passion_fruit_iced_syrup', 1000.00, '2025-07-08 12:43:57.373636'),
(19, 'syrups', 'ice_tea_syrup', 1000.00, '2025-07-08 12:43:57.463725'),
(20, 'syrups', 'white_chocolate_sauce', 1000.00, '2025-07-08 12:43:57.564843'),
(21, 'syrups', 'caramel_sauce', 1000.00, '2025-07-08 12:43:57.661456'),
(22, 'syrups', 'condense_milk_sauce', 200.00, '2025-07-08 12:43:57.734225'),
(23, 'premixes', 'mocha_frappe', 3000.00, '2025-07-08 12:43:57.858503'),
(24, 'premixes', 'chocolate_frappe', 3000.00, '2025-07-08 12:43:57.997022'),
(25, 'premixes', 'half_and_half', 2000.00, '2025-07-08 12:43:58.164684');

--
-- Name: inventory_id_seq; Type: SEQUENCE SET; Schema: public; Owner: validation_user
--

SELECT pg_catalog.setval('public.inventory_id_seq', 25, true);

--
-- Name: inventory inventory_ingredient_type_subtype_key; Type: CONSTRAINT; Schema: public; Owner: validation_user
--

ALTER TABLE ONLY public.inventory
    ADD CONSTRAINT inventory_ingredient_type_subtype_key UNIQUE (ingredient_type, subtype);

--
-- Name: inventory inventory_pkey; Type: CONSTRAINT; Schema: public; Owner: validation_user
--

ALTER TABLE ONLY public.inventory
    ADD CONSTRAINT inventory_pkey PRIMARY KEY (id);