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
(1, 'coffee_beans', 'regular', 4532.00, '2025-06-18 11:06:35.746233'),
(2, 'coffee_beans', 'decaf', 2785.00, '2025-05-31 18:54:50.54523'),
(3, 'cups', 'H7', 30.00, '2025-05-31 19:04:37.781378'),
(4, 'cups', 'H9', 286.00, '2025-06-18 11:06:35.938982'),
(5, 'cups', 'H12', 30.00, '2025-05-31 19:04:37.781378'),
(6, 'cups', 'C7', 30.00, '2025-05-31 19:04:37.781378'),
(7, 'cups', 'C9', 300.00, '2025-06-17 18:33:49.904237'),
(8, 'cups', 'C12', 30.00, '2025-05-31 19:04:37.781378'),
(9, 'cups', 'C16', 30.00, '2025-05-31 19:04:37.781378'),
(10, 'milk', 'whole', 11550.00, '2025-06-18 11:06:35.872966'),
(11, 'milk', 'skim', 1000.00, '2025-05-31 19:05:33.94022'),
(12, 'milk', 'oat', 8000.00, '2025-06-17 18:33:49.719375'),
(13, 'milk', 'soy', 1000.00, '2025-05-31 19:05:33.94022'),
(14, 'syrup', 'vanilla', 3000.00, '2025-06-17 18:33:49.816143'),
(15, 'syrup', 'caramel', 100.00, '2025-05-31 19:06:39.597242'),
(16, 'syrup', 'hazelnut', 100.00, '2025-05-31 19:06:39.597242');

--
-- Name: inventory_id_seq; Type: SEQUENCE SET; Schema: public; Owner: validation_user
--

SELECT pg_catalog.setval('public.inventory_id_seq', 16, true);

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